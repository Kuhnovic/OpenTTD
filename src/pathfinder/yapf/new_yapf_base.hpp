/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <http://www.gnu.org/licenses/>.
 */

/** @file yapf_base.hpp Base classes for YAPF. */

#ifndef NEW_YAPF_BASE_HPP
#define NEW_YAPF_BASE_HPP

// TO DO LIST:
// - Investigate static vs non-static, it seems to make a difference. See amount of discarded nodes in lots_of_ships
// - Shortcut if destination == origin?
// - Be careful with the best_intermediate_node, it might be nullptr, don't take a ref
// - FInd a way to ensure the static member isn't reused...
// - Make sure you can only run the pathfinder instance once
// - Check the closed map assert in yapf_base, it's quite good
// - COnsider shrinking the node array

#include <queue>
#include <format>
#include <iostream>
#include <unordered_map>
#include <cassert>

#include "flat_hash_map.hpp"

enum class NodeState : unsigned char {
	Open, Closed
};
enum class AddNodeResult : unsigned char {
	AddedNew, UpdatedBetterCost, RejectedNodeClosed, RejectedWorseCost, RejectedListFull
};

template<typename TNodeData>
class NodeStorage {
public:
	struct Node {
		TNodeData data;
		NodeState state;
		const Node *parent_node;
		int movement_cost;
		int heuristic;
	};

private:
	using THash = decltype(std::declval<TNodeData>().GetHash());

	struct HeapElement {
		std::reference_wrapper<Node> node;
		int original_total_cost;

		std::strong_ordering operator <=>(const HeapElement &other) const
		{
			/* The standard does not specify which std::priority_map element has priority in case of equal priority (cost).
			 * Since the nodes are stored in a std::vector we can use the element's address as a tie-breaker. */
			if (original_total_cost == other.original_total_cost) return &this->node <=> &other.node;
			return original_total_cost <=> other.original_total_cost;
		}
	};


	class BinaryHeap : public std::priority_queue<HeapElement, std::vector<HeapElement>, std::greater<>> {
	public:
		void Clear()
		{
			/* A bit of hack, but necessary to be able to clear the binary heap. */
			this->c.clear();
		}
	};

	std::vector<Node> nodes;
	BinaryHeap open_queue;

	std::array<Node *, 5120> node_lookup;
	std::unordered_map<THash, Node *> node_lookup_collisions;

	size_t node_limit = 0;

public:
	// CHange to constructor argument
	void SetNodeLimit(size_t limit)
	{
		assert(this->nodes.empty()); // Changing the limit could cause the vector resize, resulting in dangling node refs.
		this->nodes.reserve(limit);
		this->node_limit = limit;
	}

	void Clear()
	{
		this->open_queue.Clear();
		this->nodes.clear();
		memset(this->node_lookup.data(), 0, this->node_lookup.size() * sizeof(decltype(this->node_lookup)::value_type));
		this->node_lookup_collisions.clear();
	}

	AddNodeResult AddOpenNode(TNodeData &&node, const Node *parent_node, int movement_cost, int heuristic)
	{
		assert(this->node_limit > 0); // Ensure the node limit was set

		// Prevent exceeding the capacity of the vector, as this would cause a reallocation of the internal array.
		// This in turn would render all pointers to nodes (e.g. parent node pointers) invalid, which would be bad.
		if (nodes.size() + 1 > node_limit) {
			//std::cout << "NODE LIMIT REACHED " << nodes.capacity() << "\n";
			return AddNodeResult::RejectedListFull;
		}

		// TODO check if these asserts are really that meaningful, especially the second one
		assert(parent_node == nullptr || movement_cost > parent_node->movement_cost);
		//assert(parent_node == nullptr || movement_cost + heuristic <= parent_node->movement_cost + parent_node->heuristic);

		const THash hash = node.GetHash();
		const int index = hash * 17 % node_limit;

		Node *existing_node = nullptr;
		Node *lookup_node = node_lookup[index];
		if (lookup_node != nullptr) {
			if (lookup_node->data.GetHash() == hash) {
				existing_node = lookup_node;
			} else {
				auto it = node_lookup_collisions.find(hash);
				if (it != node_lookup_collisions.end()) existing_node = it->second;
			}
		}

		if (existing_node != nullptr) {
			if (existing_node->state == NodeState::Closed) return AddNodeResult::RejectedNodeClosed;

			if (existing_node->movement_cost <= movement_cost) {
				return AddNodeResult::RejectedWorseCost;
			}

			//return AddNodeResult::RejectedWorseCost; // TODO KB for now, until below is fixed

			//std::cout << std::format("Replacing existing node, movement cost {} is lower than {}\n", movement_cost, existing_node.movement_cost);


			// Update data of existing node in the node storage. Note that the node_map doesn't need to be updated.
			// TODO ensure entry in node_map is correct
			// TODO ensure cost is really lower?
			*existing_node = Node{ std::move(node), NodeState::Open, parent_node, movement_cost, heuristic };
			open_queue.emplace(HeapElement{ *existing_node , movement_cost + heuristic });
			return AddNodeResult::UpdatedBetterCost;
		}

		// TODO does this overwrite or not? It should!
		Node &new_node = nodes.emplace_back(Node{ std::move(node), NodeState::Open, parent_node, movement_cost, heuristic });
		//node_map.emplace(hash, new_node);

		if (lookup_node == nullptr) {
			node_lookup[index] = &new_node;
		} else {
			node_lookup_collisions.emplace(hash, &new_node);
		}

		open_queue.emplace(HeapElement{ new_node , movement_cost + heuristic });
		return AddNodeResult::AddedNew;
	}

	const Node *CloseAndGetBestNode()
	{
		// TODO let this return a reference?
		while (!open_queue.empty()) {
			HeapElement best_heap_element = open_queue.top();
			open_queue.pop();
			Node &best_node = best_heap_element.node.get();

			// If the total cost doesn't match the original cost, then we have an obsolete queue entry that has been
			// replaced by a better one at some point. We can ignore the obsolete entry and continue.
			if (best_node.movement_cost + best_node.heuristic != best_heap_element.original_total_cost) {
				assert(best_node.state == NodeState::Closed);
				assert(best_node.movement_cost + best_node.heuristic < best_heap_element.original_total_cost);
				//std::cout << std::format("Skipping obsolete node with original cost {}, current cost is {}\n", best_heap_element.original_total_cost, best_node.movement_cost + best_node.heuristic);
				continue;
			}

			assert(best_node.state == NodeState::Open);
			best_node.state = NodeState::Closed;
			return &best_node;
		}
		return nullptr;
	}

	size_t Size() const
	{
		return this->nodes.size();
	}

	bool Full() const
	{
		return this->nodes.size() >= this->node_limit;
	}

	struct NodeListStats {
		size_t number_of_lookup_collisions;
	};

	NodeListStats GetStats()
	{
		return { this->node_lookup_collisions.size()};
	}
};














template<typename TNodeData>
class NewYapfBase {
public:
	using TNodeHash = decltype(std::declval<TNodeData>().GetHash());
	using Node = NodeStorage<TNodeData>::Node;

private:
	inline static NodeStorage<TNodeData> nodes;
	inline static int instance_counter = 0; // TODO atomic?

	bool has_run = false;
	const Node *best_intermediate_node = nullptr;

	std::array<int, 5> add_node_stats{};
	inline static std::array<int, 5> add_node_stats_summed{};
	inline static int number_of_runs = 0;
	inline static int average_collisions;

protected:
	void AddNode(TNodeData &&node_data, const Node &parent, int movement_cost, int heuristic)
	{
		const AddNodeResult result = nodes.AddOpenNode(std::move(node_data), &parent, movement_cost, heuristic);
		this->add_node_stats[(int)result]++;
		this->add_node_stats_summed[(int)result]++;

	}

	void AddStartNode(TNodeData &&node_data, int movement_cost, int heuristic)
	{
		const AddNodeResult result = nodes.AddOpenNode(std::move(node_data), nullptr, movement_cost, heuristic);
		this->add_node_stats[(int)result]++;
		this->add_node_stats_summed[(int)result]++;
	}

	struct PathfinderResult {
		const bool path_found;
		const Node &best_node;
	};

	virtual bool IsNodeDestination(const Node &node) const = 0;
	virtual void ExpandNode(const Node &parent_node) = 0;

public:
	NewYapfBase(size_t node_limit)
	{
		this->instance_counter++;
		assert(instance_counter == 1);
		this->nodes.Clear();
		this->nodes.SetNodeLimit(node_limit);
		this->number_of_runs++; // TODO weird?
	}

	virtual ~NewYapfBase()
	{
		this->instance_counter--;
		this->average_collisions += this->nodes.GetStats().number_of_lookup_collisions;
	}

	void PrintStats()
	{
		assert(has_run);

		int total_add_node_calls = 0;
		for (auto &item : add_node_stats) total_add_node_calls += item;


		fmt::println("=======================================");
		fmt::println("Total calls to add node {}", total_add_node_calls);
		fmt::println("Rejected because list full {}", add_node_stats[(int)AddNodeResult::RejectedListFull]);
		fmt::println("Nodes added {}", add_node_stats[(int)AddNodeResult::AddedNew]);
		fmt::println("Nodes revisited but already closed {}", add_node_stats[(int)AddNodeResult::RejectedNodeClosed]);
		fmt::println("Nodes revisited but worse cost {}", add_node_stats[(int)AddNodeResult::RejectedWorseCost]);
		fmt::println("Nodes revisited cost updated {}", add_node_stats[(int)AddNodeResult::UpdatedBetterCost]);

		fmt::println("Number of lookup collisions {}", nodes.GetStats().number_of_lookup_collisions);
	}

	static std::string GetAverageStats()
	{
		return std::format("Average lookup collisions {}", average_collisions);
	}

	PathfinderResult FindPath()
	{
		assert(!has_run);
		has_run = true;

		for(;;) {
			const Node *current_node = nodes.CloseAndGetBestNode();
			if (current_node == nullptr) break;
			best_intermediate_node = current_node;

			// Stop search if the current node is a destination node.
			if (IsNodeDestination(*current_node)) {
				return PathfinderResult{ true, *current_node };
			}

			// Expand the search around the current node.
			ExpandNode(*current_node);
		}

		// No path found
		return PathfinderResult{ false, *best_intermediate_node };
	}
};

#endif /* NEW_YAPF_BASE_HPP */
