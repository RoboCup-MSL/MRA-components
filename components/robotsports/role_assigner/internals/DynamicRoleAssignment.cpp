
#include "DynamicRoleAssignment.h"
#include <map>
using namespace std;
using namespace MRA;

//---------------------------------------------------------------------------------------------------------------------
// assign dynamic roles to available players
void DynamicRoleAssignment::assignDynamicRolesToPlayers(const vector<dynamic_role_e>& teamFormation, std::vector<TeamPlannerRobot>& Team) {
	// sort players on id by put them in a map. Map is sorted by the key (= robotId)
	std::map <long, unsigned> idmap;
	for (unsigned idx = 0; idx < Team.size(); idx++) {
	    idmap[Team[idx].robotId] = idx;
	}

	for (unsigned dr_idx = 0; dr_idx < teamFormation.size(); dr_idx++) {
		bool role_assigned = false;
		// iterate over Team sorted by robotId (using map)
		for( std::map<long, unsigned>::iterator iter = idmap.begin(); role_assigned == false && iter != idmap.end(); ++iter ) {
			unsigned robotIdx = iter->second;
			if (teamFormation[dr_idx] == dr_GOALKEEPER) {
				if (Team[robotIdx].player_type == player_type_e::GOALIE) {
					role_assigned = true;
					Team[robotIdx].result.dynamic_role = dr_GOALKEEPER;
				}
			}
			else {
				// not goalie
				if ((Team[robotIdx].player_type != player_type_e::GOALIE) && Team[robotIdx].result.dynamic_role == dr_NONE) {
					role_assigned = true;
					Team[robotIdx].result.dynamic_role = teamFormation[dr_idx];
				}
			}
		}
	}
}
