/*
 * TeamFormation.cpp
 *
 *  Created on: Sep 11, 2016
 *      Author: jurge
 */

#include "TeamFormation.h"
using namespace MRA;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------

vector<dynamic_role_e> TeamFormation::selectTeamFormation(team_formation_e team_formation, game_state_e gamestate,
                                                          ball_status_e ball_status, const TeamPlannerParameters& plannerOptions) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    // inputs which can be used:
    //    m_plannerOptions;
    //    m_ballControlledByTeam
    //    m_ball_pickup_position;
    //     //    Game-stats (score)   [Future]
    if (gamestate == PENALTY_SHOOTOUT_AGAINST) {
        // formation strategy independent formation
        // all must be defender
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
    }
    else if (gamestate == PENALTY_SHOOTOUT) {
        // formation strategy independent formation
        formation.push_back(dynamic_role_e::dr_PENALTY_KICKER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
        formation.push_back(dynamic_role_e::dr_PENALTY_DEFENDER);
    }
    else {
        switch (team_formation){
        case FORMATION_ATTACK_SUPPORT_ONLY:
            formation = roleOnlyFormation(dr_ATTACKSUPPORTER);
            break;
        case FORMATION_DEFENDER_ONLY:
            formation = roleOnlyFormation(dr_DEFENDER);
            break;
        case FORMATION_INTERCEPTOR_ONLY:
            formation = roleOnlyFormation(dr_INTERCEPTOR);
            break;
        case FORMATION_SWEEPER_ONLY:
            formation = roleOnlyFormation(dr_SWEEPER);
            break;
        case FORMATION_SETPLAY_RECEIVER_ONLY:
            formation = roleOnlyFormation(dr_SETPLAY_RECEIVER);
            break;
        case FORMATION_SETPLAY_KICKER_ONLY:
            formation = roleOnlyFormation(dr_SETPLAY_KICKER);
            break;
        case FORMATION_BALLPLAYER_ONLY:
            formation = roleOnlyFormation(dr_BALLPLAYER);
            break;
        case FORMATION_SEARCHFORBALL_ONLY:
            formation = roleOnlyFormation(dr_SEARCH_FOR_BALL);
            break;
        case FORMATION_BEGINPOSITION_ONLY:
            formation = roleOnlyFormation(dr_BEGIN_POSITION);
            break;
        case FORMATION_PARKING_ONLY:
            formation = roleOnlyFormation(dr_PARKING);
            break;
        case FORMATION_PENALTYKICKER_ONLY:
            formation = roleOnlyFormation(dr_PENALTY_KICKER);
            break;
        case FORMATION_PENALTY_SHOOTOUT:
            formation = roleOnlyFormation(dr_PENALTY_DEFENDER);
            break;
        case FORMATION_112:
            formation = getFormation112(gamestate, ball_status, plannerOptions);
            break;
        case FORMATION_211:
            formation = getFormation211(gamestate, ball_status, plannerOptions);
            break;
        case FORMATION_013:
            formation = getFormation013(gamestate, ball_status, plannerOptions);
            break;
        case FORMATION_310:
        default:
            formation = getFormation310(gamestate, ball_status, plannerOptions);
        }
    }
    return formation;
}


std::vector<dynamic_role_e> TeamFormation::roleOnlyFormation(dynamic_role_e dynamic_role) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    formation.push_back(dynamic_role);
    formation.push_back(dynamic_role);
    formation.push_back(dynamic_role);
    formation.push_back(dynamic_role);
    return formation;
}


vector<dynamic_role_e> TeamFormation::getFormation013(game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& plannerOptions) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    switch (gamestate) {
    case FREEKICK: // intentional fall through
    case GOALKICK: // intentional fall through
    case THROWIN: // intentional fall through
    case CORNER: // intentional fall through
    case KICKOFF:  // intentional fall through
        formation.push_back(dynamic_role_e::dr_SETPLAY_RECEIVER);
        formation.push_back(dynamic_role_e::dr_SETPLAY_KICKER);
        formation.push_back(dynamic_role_e::dr_SWEEPER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        break;
    case PENALTY: // intentional fall through
        formation.push_back(dynamic_role_e::dr_PENALTY_KICKER);
        formation.push_back(dynamic_role_e::dr_SWEEPER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        break;
    case NONE:   // intentional fall through
    case NORMAL: // intentional fall through
    case NORMAL_ATTACK: // intentional fall through
    case NORMAL_DEFEND: // intentional fall through
    case YELLOW_CARD_AGAINST: // intentional fall through
    case RED_CARD_AGAINST: // intentional fall through
    case GOAL: // intentional fall through
    case GOAL_AGAINST: // intentional fall through
        switch (ball_status)
        {
            case ball_status_e::OWNED_BY_PLAYER:
            case ball_status_e::OWNED_BY_TEAMMATE:
                formation.push_back(dynamic_role_e::dr_BALLPLAYER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            case ball_status_e::OWNED_BY_TEAM:
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            case ball_status_e::FREE:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            default:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
        }
        break;
    case PARKING: // intentional fall through
    case BEGIN_POSITION: // intentional fall through
    case KICKOFF_AGAINST: // intentional fall through
    case FREEKICK_AGAINST:  // intentional fall through
    case GOALKICK_AGAINST: // intentional fall through
    case THROWIN_AGAINST: // intentional fall through
    case CORNER_AGAINST: // intentional fall through
    case PENALTY_AGAINST: // intentional fall through
    case DROPPED_BALL: // intentional fall through
    case PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            case ball_status_e::OWNED_BY_PLAYER:
            case ball_status_e::OWNED_BY_TEAMMATE:
                formation.push_back(dynamic_role_e::dr_BALLPLAYER);
                break;
            default:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
        }
        formation.push_back(dynamic_role_e::dr_SWEEPER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        break;
    default:
        ;// empty
    };
    return formation;
}

vector<dynamic_role_e> TeamFormation::getFormation112(game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& plannerOptions) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    switch (gamestate) {
    case FREEKICK: // intentional fall through
    case GOALKICK: // intentional fall through
    case THROWIN: // intentional fall through
    case CORNER: // intentional fall through
    case KICKOFF:  // intentional fall through
        formation.push_back(dynamic_role_e::dr_SETPLAY_RECEIVER);
        formation.push_back(dynamic_role_e::dr_SETPLAY_KICKER);
        if (plannerOptions.no_sweeper_during_setplay) {
            formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        }
        else {
            formation.push_back(dynamic_role_e::dr_SWEEPER);
        }
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    case PENALTY:
        formation.push_back(dynamic_role_e::dr_PENALTY_KICKER);
        formation.push_back(dynamic_role_e::dr_SWEEPER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_DEFENDER);
        break;
    case NONE:   // intentional fall through
    case NORMAL: // intentional fall through
    case NORMAL_ATTACK: // intentional fall through
    case NORMAL_DEFEND: // intentional fall through
    case PARKING: // intentional fall through
    case BEGIN_POSITION: // intentional fall through
    case KICKOFF_AGAINST: // intentional fall through
    case FREEKICK_AGAINST:  // intentional fall through
    case GOALKICK_AGAINST: // intentional fall through
    case THROWIN_AGAINST: // intentional fall through
    case CORNER_AGAINST: // intentional fall through
    case PENALTY_AGAINST: // intentional fall through
    case DROPPED_BALL: // intentional fall through
    case YELLOW_CARD_AGAINST: // intentional fall through
    case RED_CARD_AGAINST: // intentional fall through
    case GOAL: // intentional fall through
    case GOAL_AGAINST: // intentional fall through
    case PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            case ball_status_e::OWNED_BY_PLAYER:
            case ball_status_e::OWNED_BY_TEAMMATE:
                formation.push_back(dynamic_role_e::dr_BALLPLAYER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            case ball_status_e::OWNED_BY_TEAM:
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            case ball_status_e::FREE:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                break;
            default:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_DEFENDER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        }
        break;
    default:
        ;// empty
    };
    return formation;
}

vector<dynamic_role_e> TeamFormation::getFormation211(game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& plannerOptions) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    switch (gamestate) {
    case FREEKICK: // intentional fall through
    case GOALKICK: // intentional fall through
    case THROWIN: // intentional fall through
    case CORNER: // intentional fall through
    case KICKOFF:
        formation.push_back(dynamic_role_e::dr_SETPLAY_RECEIVER);
        formation.push_back(dynamic_role_e::dr_SETPLAY_KICKER);
        if (plannerOptions.no_sweeper_during_setplay) {
            formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        }
        else {
            formation.push_back(dynamic_role_e::dr_SWEEPER);
        }
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    case PENALTY:
        formation.push_back(dynamic_role_e::dr_PENALTY_KICKER);
        formation.push_back(dynamic_role_e::dr_SWEEPER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    case NONE:   // intentional fall through
    case NORMAL: // intentional fall through
    case NORMAL_ATTACK: // intentional fall through
    case NORMAL_DEFEND: // intentional fall through
    case PARKING: // intentional fall through
    case BEGIN_POSITION: // intentional fall through
    case KICKOFF_AGAINST: // intentional fall through
    case FREEKICK_AGAINST:  // intentional fall through
    case GOALKICK_AGAINST: // intentional fall through
    case THROWIN_AGAINST: // intentional fall through
    case CORNER_AGAINST: // intentional fall through
    case PENALTY_AGAINST: // intentional fall through
    case DROPPED_BALL: // intentional fall through
    case YELLOW_CARD_AGAINST: // intentional fall through
    case RED_CARD_AGAINST: // intentional fall through
    case GOAL: // intentional fall through
    case GOAL_AGAINST: // intentional fall through
    case PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            // create pass possibility if two players are left, only when we control the ball (see issue #218)
            case ball_status_e::OWNED_BY_PLAYER:
            case ball_status_e::OWNED_BY_TEAMMATE:
                formation.push_back(dynamic_role_e::dr_BALLPLAYER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                break;
            case ball_status_e::OWNED_BY_TEAM:
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                break;
            case ball_status_e::FREE:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                break;
            default:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
                formation.push_back(dynamic_role_e::dr_SWEEPER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        }
        break;
    default:
        ;// empty
    };
    return formation;
}

vector<dynamic_role_e> TeamFormation::getFormation310(game_state_e gamestate, ball_status_e ball_status, const TeamPlannerParameters& plannerOptions) {
    vector<dynamic_role_e> formation = vector<dynamic_role_e>();
    switch (gamestate) {
    case FREEKICK: // intentional fall through
    case GOALKICK: // intentional fall through
    case THROWIN: // intentional fall through
    case CORNER: // intentional fall through
    case KICKOFF:  // intentional fall through
        formation.push_back(dynamic_role_e::dr_SETPLAY_RECEIVER);
        formation.push_back(dynamic_role_e::dr_SETPLAY_KICKER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    case PENALTY: // intentional fall through
        formation.push_back(dynamic_role_e::dr_PENALTY_KICKER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    case NONE:   // intentional fall through
    case NORMAL: // intentional fall through
    case NORMAL_ATTACK: // intentional fall through
    case NORMAL_DEFEND: // intentional fall through
    case PARKING: // intentional fall through
    case BEGIN_POSITION: // intentional fall through
    case KICKOFF_AGAINST: // intentional fall through
    case FREEKICK_AGAINST:  // intentional fall through
    case GOALKICK_AGAINST: // intentional fall through
    case THROWIN_AGAINST: // intentional fall through
    case CORNER_AGAINST: // intentional fall through
    case PENALTY_AGAINST: // intentional fall through
    case DROPPED_BALL: // intentional fall through
    case YELLOW_CARD_AGAINST: // intentional fall through
    case RED_CARD_AGAINST: // intentional fall through
    case GOAL: // intentional fall through
    case GOAL_AGAINST: // intentional fall through
    case PENALTY_SHOOTOUT_AGAINST: // should be handled at higher level
    case PENALTY_SHOOTOUT: // should be handled at higher level
        switch (ball_status)
        {
            case ball_status_e::OWNED_BY_PLAYER:
            case ball_status_e::OWNED_BY_TEAMMATE:
                formation.push_back(dynamic_role_e::dr_BALLPLAYER);
                break;
            case ball_status_e::OWNED_BY_TEAM:
                formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
                break;
            default:
                formation.push_back(dynamic_role_e::dr_INTERCEPTOR);
        }
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        formation.push_back(dynamic_role_e::dr_ATTACKSUPPORTER);
        break;
    default:
        ;// empty
    };
    return formation;
}

