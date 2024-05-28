/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <http://www.gnu.org/licenses/>.
 */

// TODO KB description
 /** @file yapf_ship_regions.cpp Implementation of YAPF for water regions, which are used for finding intermediate ship destinations. */

#include "../../stdafx.h"
#include "../../ship.h"

#include "yapf.hpp"
#include "yapf_ship_regions.h"
#include "../water_regions.h"

#include "../../safeguards.h"


#include "new_yapf_base.hpp"


struct WaterRegionNodeData {
	int GetHash() const
	{
		return CalculateWaterRegionPatchHash(patch);
	}

	WaterRegionPatchDesc patch;
};


constexpr int DIRECT_NEIGHBOR_COST = 100;
constexpr int NODES_PER_REGION = 4;
constexpr int MAX_NUMBER_OF_NODES = 65536; // TODO dynamic based on map size?


class NewYapfShipRegions : public NewYapfBase<WaterRegionNodeData> {
private:
	const WaterRegionPatchDesc destination_patch;

	int CalculateHeuristic(const WaterRegionPatchDesc &lhs, const WaterRegionPatchDesc &rhs) const
	{
		return (std::abs(lhs.x - rhs.x) + std::abs(lhs.y - rhs.y)) * DIRECT_NEIGHBOR_COST;
	}

	bool IsNodeDestination(const Node &node) const override
	{
		return node.data.patch == destination_patch;
	}

	void ExpandNode(const Node &parent_node) override
	{
		//fmt::println("REGION NEW: expanding from {} {} {} - G={}, H={}", parent_node.data.patch.x, parent_node.data.patch.y, parent_node.data.patch.label, parent_node.movement_cost, parent_node.heuristic);
		TVisitWaterRegionPatchCallBack visitFunc = [&](const WaterRegionPatchDesc &water_region_patch)
		{
			const int movement_cost = parent_node.movement_cost + DIRECT_NEIGHBOR_COST;
			const int heuristic = CalculateHeuristic(water_region_patch, destination_patch);
			AddNode(WaterRegionNodeData{ water_region_patch }, parent_node, movement_cost, heuristic);
			//fmt::println(" -> adding {} {} {} - G={}, H={}", water_region_patch.x, water_region_patch.y, water_region_patch.label, movement_cost, heuristic);
		};
		VisitWaterRegionPatchNeighbors(parent_node.data.patch, visitFunc);
	}

public:
	explicit NewYapfShipRegions(WaterRegionPatchDesc start_patch) : NewYapfBase(MAX_NUMBER_OF_NODES), destination_patch(std::move(start_patch))
	{
	}

	void AddOriginRegion(WaterRegionPatchDesc patch)
	{
		AddStartNode(WaterRegionNodeData{ patch }, 0, CalculateHeuristic(patch, destination_patch));
	}
};

std::vector<WaterRegionPatchDesc> YapfShipFindWaterRegionPathNEW(const Ship *v, TileIndex start_tile, int max_returned_path_length)
{
	const WaterRegionPatchDesc start_water_region_patch = GetWaterRegionPatchInfo(start_tile);

	NewYapfShipRegions pathfinder{ start_water_region_patch };

	if (v->current_order.IsType(OT_GOTO_STATION)) {
		DestinationID station_id = v->current_order.GetDestination();
		const BaseStation *station = BaseStation::Get(station_id);
		TileArea tile_area;
		station->GetTileArea(&tile_area, STATION_DOCK);
		for (const auto &tile : tile_area) {
			if (IsDockingTile(tile) && IsShipDestinationTile(tile, station_id)) {
				pathfinder.AddOriginRegion(GetWaterRegionPatchInfo(tile));
			}
		}
	} else {
		pathfinder.AddOriginRegion(GetWaterRegionPatchInfo(v->dest_tile));
	}

	
		/* If origin and destination are the same we simply return that water patch. */ // TODO this is not done, is it needed?
	std::vector<WaterRegionPatchDesc> path = { start_water_region_patch };
	path.reserve(max_returned_path_length);

	/* Find best path. */
	auto pathfinder_result = pathfinder.FindPath();
	if (!pathfinder_result.path_found) return {}; // Path not found.

	auto* node = &pathfinder_result.best_node;
	for (int i = 0; i < max_returned_path_length - 1; ++i) {
		if (node != nullptr) {
			node = node->parent_node;
			if (node != nullptr) path.push_back(node->data.patch);
		}
	}

	assert(!path.empty());
	return path;
}
