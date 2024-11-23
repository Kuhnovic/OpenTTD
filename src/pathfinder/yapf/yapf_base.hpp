/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <http://www.gnu.org/licenses/>.
 */

/** @file yapf_base.hpp Base classes for YAPF. */

#ifndef YAPF_BASE_HPP
#define YAPF_BASE_HPP

#include "../../debug.h"
#include "../../settings_type.h"
#include "../../misc/dbg_helpers.h"
#include "yapf_type.hpp"

#define BENCHMARK_ENABLED
#define BENCHMARK_GROUP_RESULTS_PER_TRANSPORT_TYPE // Disable to show distinct results for each individual YAPF template instantation

#ifdef BENCHMARK_ENABLED

#include "error_func.h"
#include "3rdparty/nanobench/nanobench.h"

struct BenchmarkStats {
	int runs = 0;
	int skipped = 0;
	double summed_reference_duration = 0;
	double summed_alternative_duration = 0;
};
#ifdef BENCHMARK_GROUP_RESULTS_PER_TRANSPORT_TYPE
static std::unordered_map<char, BenchmarkStats> _benchmark_results;
#endif
#endif

/**
 * CYapfBaseT - A-star type path finder base class.
 *  Derive your own pathfinder from it. You must provide the following template argument:
 *    Types      - used as collection of local types used in pathfinder
 *
 * Requirements for the Types struct:
 *  ----------------------------------
 *  The following types must be defined in the 'Types' argument:
 *    - Types::Tpf - your pathfinder derived from CYapfBaseT
 *    - Types::NodeList - open/closed node list (look at CNodeList_HashTableT)
 *  NodeList needs to have defined local type Titem - defines the pathfinder node type.
 *  Node needs to define local type Key - the node key in the collection ()
 *
 *  For node list you can use template class NodeList, for which
 *  you need to declare only your node type. Look at test_yapf.h for an example.
 *
 *
 *  Requirements to your pathfinder class derived from CYapfBaseT:
 *  --------------------------------------------------------------
 *  Your pathfinder derived class needs to implement following methods:
 *    inline void PfSetStartupNodes()
 *    inline void PfFollowNode(Node &org)
 *    inline bool PfCalcCost(Node &n)
 *    inline bool PfCalcEstimate(Node &n)
 *    inline bool PfDetectDestination(Node &n)
 *
 *  For more details about those methods, look at the end of CYapfBaseT
 *  declaration. There are some examples. For another example look at
 *  test_yapf.h (part or unittest project).
 */
template <class Types>
class CYapfBaseT {
public:
	typedef typename Types::Tpf Tpf; ///< the pathfinder class (derived from THIS class)
	typedef typename Types::TrackFollower TrackFollower;
	typedef typename Types::NodeList NodeList; ///< our node list
	typedef typename Types::VehicleType VehicleType; ///< the type of vehicle
	typedef typename NodeList::Item Node; ///< this will be our node type
	typedef typename Node::Key Key; ///< key to hash tables

	NodeList nodes; ///< node list multi-container

protected:
	Node *best_dest_node = nullptr; ///< pointer to the destination node found at last round
	Node *best_intermediate_node = nullptr; ///< here should be node closest to the destination if path not found
	const YAPFSettings *settings; ///< current settings (_settings_game.yapf)
	int max_search_nodes; ///< maximum number of nodes we are allowed to visit before we give up
	const VehicleType *vehicle = nullptr; ///< vehicle that we are trying to drive

	int stats_cost_calcs = 0; ///< stats - how many node's costs were calculated
	int stats_cache_hits = 0; ///< stats - how many node's costs were reused from cache

	static inline auto last_print_time = std::chrono::steady_clock::now();

public:
	int num_steps = 0; ///< this is there for debugging purposes (hope it doesn't hurt)

#ifdef BENCHMARK_ENABLED
	bool use_alternative_implementation;
#else
	static constexpr bool use_alternative_implementation = false;
#endif

public:
	/** default constructor */
	inline CYapfBaseT() : settings(&_settings_game.pf.yapf), max_search_nodes(PfGetSettings().max_search_nodes) {}

	/** default destructor */
	~CYapfBaseT() {}

protected:
	/** to access inherited path finder */
	inline Tpf &Yapf()
	{
		return *static_cast<Tpf *>(this);
	}

public:
	/** return current settings (can be custom - company based - but later) */
	inline const YAPFSettings &PfGetSettings() const
	{
		return *this->settings;
	}

#ifdef BENCHMARK_ENABLED
	std::vector<int> GetPathHashes()
	{
		std::vector<int> result;
		Node *best_node = GetBestNode();
		while (best_node != nullptr) {
			int hash = best_node->GetKey().CalcHash();
			result.emplace_back(hash);
			best_node = best_node->parent;
		}
		return result;
	}

	void RunBenchmark(const VehicleType *v)
	{
		using namespace std::chrono_literals;

#ifndef BENCHMARK_GROUP_RESULTS_PER_TRANSPORT_TYPE
		static std::unordered_map<char, BenchmarkStats> _benchmark_results;
#endif
		const char ttc = Yapf().TransportTypeChar();
		auto &results = _benchmark_results[ttc];
		++results.runs;

		ankerl::nanobench::Bench bench;
		bench.output(nullptr);
		bench.warmup(1); // Update water regions, warm up caches, etc
		bench.relative(true);
		//bench.minEpochIterations(10);
		//bench.minEpochTime(10ms);

		bench.run("reference", [&] {
			this->use_alternative_implementation = false;
			FindPathInternal(v);
		});
		const auto reference_result = bench.results().back();
		const auto reference_path = GetPathHashes();

		bench.run("alternative", [&] {
#ifdef BENCHMARK_COMPARE_SAME_IMPLEMENTATION
			this->use_alternative_implementation = false;
#else
			this->use_alternative_implementation = true;
#endif
			FindPathInternal(v);
		});
		const auto alternative_result = bench.results().back();
		const auto alternative_path = GetPathHashes();

		/* Ensure both versions result in the same path */
		if (reference_path.size() != alternative_path.size()) FatalError("YAPF Benchmark: paths are of different length");
		for (int i = 0; i < reference_path.size() ; ++i) {
			if (reference_path.at(i) != alternative_path.at(i)) FatalError("YAPF Benchmark: paths are different");
		}

		/* Skip unstable results */
		constexpr double error_limit = 0.05;
		constexpr auto elapsed = ankerl::nanobench::Result::Measure::elapsed;
		if (reference_result.medianAbsolutePercentError(elapsed) > error_limit
			|| alternative_result.medianAbsolutePercentError(elapsed) > error_limit) {
			++results.skipped;
			return;
		}

		results.summed_reference_duration += reference_result.average(elapsed);
		results.summed_alternative_duration += alternative_result.average(elapsed);

		const double n = results.runs;
		const double ratio = results.summed_alternative_duration / results.summed_reference_duration;
		const double percentage_faster = (1.0 - ratio) * 100.0;
		if (std::chrono::steady_clock::now() - this->last_print_time > 1000ms) {
			this->last_print_time = std::chrono::steady_clock::now();
			Debug(yapf, 0, "{} {} : runs = {}, results skipped = {} ({:.1f}%), ratio = {:.4f} ({:.2f}% {})",
				static_cast<void*>(this),
				ttc,
				n,
				results.skipped, (results.skipped / n) * 100.0,
				ratio, std::abs(percentage_faster), (percentage_faster >= 0.0 ? "faster" : "slower"));
		}

	}
#endif

	/**
	 * Main pathfinder routine:
	 *   - set startup node(s)
	 *   - main loop that stops if:
	 *      - the destination was found
	 *      - or the open list is empty (no route to destination).
	 *      - or the maximum amount of loops reached - max_search_nodes (default = 10000)
	 * @return true if the path was found
	 */
	inline bool FindPath(const VehicleType *v)
	{
#ifdef BENCHMARK_ENABLED
		RunBenchmark(v);
		return FindPathInternal(v);
	}

	inline bool FindPathInternal(const VehicleType *v)
	{
		// Reset members for next run
		this->nodes = {};
		this->best_dest_node = nullptr;
		this->best_intermediate_node = nullptr;
		this->stats_cost_calcs = 0;
		this->stats_cache_hits = 0;
		this->num_steps = 0;
#endif

		this->vehicle = v;

		Yapf().PfSetStartupNodes();

		for (;;) {
			this->num_steps++;
			Node *best_open_node = this->nodes.GetBestOpenNode();
			if (best_open_node == nullptr) break;

			if (Yapf().PfDetectDestination(*best_open_node)) {
				this->best_dest_node = best_open_node;
				break;
			}

			Yapf().PfFollowNode(*best_open_node);
			if (this->max_search_nodes != 0 && this->nodes.ClosedCount() >= this->max_search_nodes) break;

			this->nodes.PopOpenNode(best_open_node->GetKey());
			this->nodes.InsertClosedNode(*best_open_node);
		}

		const bool destination_found = (this->best_dest_node != nullptr);

		if (_debug_yapf_level >= 3) {
			const UnitID veh_idx = (this->vehicle != nullptr) ? this->vehicle->unitnumber : 0;
			const char ttc = Yapf().TransportTypeChar();
			const float cache_hit_ratio = (this->stats_cache_hits == 0) ? 0.0f : ((float)this->stats_cache_hits / (float)(this->stats_cache_hits + this->stats_cost_calcs) * 100.0f);
			const int cost = destination_found ? this->best_dest_node->cost : -1;
			const int dist = destination_found ? this->best_dest_node->estimate - this->best_dest_node->cost : -1;

			Debug(yapf, 3, "[YAPF{}]{}{:4d} - {} rounds - {} open - {} closed - CHR {:4.1f}% - C {} D {}",
				ttc, destination_found ? '-' : '!', veh_idx, this->num_steps, this->nodes.OpenCount(), this->nodes.ClosedCount(), cache_hit_ratio, cost, dist
			);
		}

		return destination_found;
	}

	/**
	 * If path was found return the best node that has reached the destination. Otherwise
	 *  return the best visited node (which was nearest to the destination).
	 */
	inline Node *GetBestNode()
	{
		return (this->best_dest_node != nullptr) ? this->best_dest_node : this->best_intermediate_node;
	}

	/**
	 * Calls NodeList::CreateNewNode() - allocates new node that can be filled and used
	 *  as argument for AddStartupNode() or AddNewNode()
	 */
	inline Node &CreateNewNode()
	{
		Node &node = this->nodes.CreateNewNode();
		return node;
	}

	/** Add new node (created by CreateNewNode and filled with data) into open list */
	inline void AddStartupNode(Node &n)
	{
		Yapf().PfNodeCacheFetch(n);
		/* insert the new node only if it is not there */
		if (this->nodes.FindOpenNode(n.key) == nullptr) {
			this->nodes.InsertOpenNode(n);
		} else {
			/* if we are here, it means that node is already there - how it is possible?
			 *   probably the train is in the position that both its ends point to the same tile/exit-dir
			 *   very unlikely, but it happened */
		}
	}

	/** add multiple nodes - direct children of the given node */
	inline void AddMultipleNodes(Node *parent, const TrackFollower &tf)
	{
		bool is_choice = (KillFirstBit(tf.new_td_bits) != TRACKDIR_BIT_NONE);
		for (TrackdirBits rtds = tf.new_td_bits; rtds != TRACKDIR_BIT_NONE; rtds = KillFirstBit(rtds)) {
			Trackdir td = (Trackdir)FindFirstBit(rtds);
			Node &n = Yapf().CreateNewNode();
			n.Set(parent, tf.new_tile, td, is_choice);
			Yapf().AddNewNode(n, tf);
		}
	}

	/**
	 * In some cases an intermediate node branch should be pruned.
	 * The most prominent case is when a red EOL signal is encountered, but
	 * there was a segment change (e.g. a rail type change) before that. If
	 * the branch would not be pruned, the rail type change location would
	 * remain the best intermediate node, and thus the vehicle would still
	 * go towards the red EOL signal.
	 */
	void PruneIntermediateNodeBranch(Node *n)
	{
		bool intermediate_on_branch = false;
		while (n != nullptr && (n->segment->end_segment_reason & ESRB_CHOICE_FOLLOWS) == 0) {
			if (n == Yapf().best_intermediate_node) intermediate_on_branch = true;
			n = n->parent;
		}
		if (intermediate_on_branch) Yapf().best_intermediate_node = n;
	}

	/**
	 * AddNewNode() - called by Tderived::PfFollowNode() for each child node.
	 *  Nodes are evaluated here and added into open list
	 */
	void AddNewNode(Node &n, const TrackFollower &tf)
	{
		if (this->use_alternative_implementation) {
			AddNewNode_ALTERNATIVE(n, tf);
		} else {
			AddNewNode_REFERENCE(n, tf);
		}
	}

	void AddNewNode_ALTERNATIVE(Node &n, const TrackFollower &tf)
	{
		/* NOTE: cost calculations can be quite heavy, especially for trains, so we make sure we do them only once! */
		bool cost_calculated = false;
		const auto calculateNodeCosts = [&]() -> bool {
			if (cost_calculated) return true;
			cost_calculated = true;

			Yapf().PfNodeCacheFetch(n) ? ++this->stats_cost_calcs : ++this->stats_cache_hits;

			if (!Yapf().PfCalcCost(n, &tf)) return false; // Node was marked as invalid
			if (!Yapf().PfCalcEstimate(n)) return false; // Node was marked as invalid
			return true;
		};

		/* Don't re-add the node if another node with the same key is already closed */
		Node *closed_node = this->nodes.FindClosedNode(n.GetKey());
		if (closed_node != nullptr) {
#ifdef WITH_ASSERT
			/* another node exists with the same key in the closed list
			 * is it better than new one? */
			if (!calculateNodeCosts()) return; // Return if node was marked as invalid during cost calculations

			if (n.GetCostEstimate() < closed_node->GetCostEstimate()) {
				/* If this assert occurs, you have probably problem in
				 * your Tderived::PfCalcCost() or Tderived::PfCalcEstimate().
				 * The problem could be:
				 *  - PfCalcEstimate() gives too large numbers
				 *  - PfCalcCost() gives too small numbers
				 *  - You have used negative cost penalty in some cases (cost bonus) */
				NOT_REACHED();
			}
#endif
			return;
		}

		if (!calculateNodeCosts()) return;  // Return if node was marked as invalid during cost calculations

		/* The new node can be set as the best intermediate node only once we're
		 * certain it will be finalized by being inserted into the open list. */
		bool set_intermediate = this->max_search_nodes > 0 && (this->best_intermediate_node == nullptr
			|| (this->best_intermediate_node->GetCostEstimate() - this->best_intermediate_node->GetCost()) > (n.GetCostEstimate() - n.GetCost()));

		/* check new node against open list */
		Node *open_node = this->nodes.FindOpenNode(n.GetKey());
		if (open_node != nullptr) {
			/* another node exists with the same key in the open list
			 * is it better than new one? */
			if (n.GetCostEstimate() < open_node->GetCostEstimate()) {
				/* update the old node by value from new one */
				this->nodes.PopOpenNode(n.GetKey());
				*open_node = n;
				/* add the updated old node back to open list */
				this->nodes.InsertOpenNode(*open_node);
				if (set_intermediate) this->best_intermediate_node = open_node;
			}
			return;
		}

		/* the new node is really new
		 * add it to the open list */
		this->nodes.InsertOpenNode(n);
		if (set_intermediate) this->best_intermediate_node = &n;
	}

	void AddNewNode_REFERENCE(Node &n, const TrackFollower &tf)
	{
		/* evaluate the node */
		bool cached = Yapf().PfNodeCacheFetch(n);
		if (!cached) {
			this->stats_cost_calcs++;
		} else {
			this->stats_cache_hits++;
		}

		bool valid = Yapf().PfCalcCost(n, &tf);

		if (valid) valid = Yapf().PfCalcEstimate(n);

		/* have the cost or estimate callbacks marked this node as invalid? */
		if (!valid) return;

		/* The new node can be set as the best intermediate node only once we're
		 * certain it will be finalized by being inserted into the open list. */
		bool set_intermediate = this->max_search_nodes > 0 && (this->best_intermediate_node == nullptr || (this->best_intermediate_node->GetCostEstimate() - this->best_intermediate_node->GetCost()) > (n.GetCostEstimate() - n.GetCost()));

		/* check new node against open list */
		Node *open_node = this->nodes.FindOpenNode(n.GetKey());
		if (open_node != nullptr) {
			/* another node exists with the same key in the open list
			 * is it better than new one? */
			if (n.GetCostEstimate() < open_node->GetCostEstimate()) {
				/* update the old node by value from new one */
				this->nodes.PopOpenNode(n.GetKey());
				*open_node = n;
				/* add the updated old node back to open list */
				this->nodes.InsertOpenNode(*open_node);
				if (set_intermediate) this->best_intermediate_node = open_node;
			}
			return;
		}

		/* check new node against closed list */
		Node *closed_node = this->nodes.FindClosedNode(n.GetKey());
		if (closed_node != nullptr) {
			/* another node exists with the same key in the closed list
			 * is it better than new one? */
			int node_est = n.GetCostEstimate();
			int closed_est = closed_node->GetCostEstimate();
			if (node_est < closed_est) {
				/* If this assert occurs, you have probably problem in
				 * your Tderived::PfCalcCost() or Tderived::PfCalcEstimate().
				 * The problem could be:
				 *  - PfCalcEstimate() gives too large numbers
				 *  - PfCalcCost() gives too small numbers
				 *  - You have used negative cost penalty in some cases (cost bonus) */
				NOT_REACHED();
			}
			return;
		}
		/* the new node is really new
		 * add it to the open list */
		this->nodes.InsertOpenNode(n);
		if (set_intermediate) this->best_intermediate_node = &n;
	}


	const VehicleType * GetVehicle() const
	{
		return this->vehicle;
	}

	void DumpBase(DumpTarget &dmp) const
	{
		dmp.WriteStructT("nodes", &this->nodes);
		dmp.WriteValue("num_steps", this->num_steps);
	}
};

#endif /* YAPF_BASE_HPP */
