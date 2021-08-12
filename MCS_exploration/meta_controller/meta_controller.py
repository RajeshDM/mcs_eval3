from MCS_exploration import sequence_generator,main #import SequenceGenerator
from MCS_exploration.navigation.bounding_box_navigator  import BoundingBoxNavigator 
from MCS_exploration.gym_ai2thor.envs.mcs_nav import McsNavWrapper
from MCS_exploration.gym_ai2thor.envs.mcs_face import McsFaceWrapper
from MCS_exploration.gym_ai2thor.envs.trophy import AGENT_RADIUS
from icecream import ic
MAX_REACH_DISTANCE = 1

def get_goal(goal_string):
    goal = goal_string.split("|")
    _, goal_x, goal_y, goal_z = goal
    goal_x = float(goal_x)
    goal_y = float(goal_y)
    goal_z = float(goal_z)
    #return (goal_x, goal_y, goal_z)
    return (goal_x,goal_z)

class MetaController:
    def __init__(self, env,level):
        self.env = env
        self.nav_env = McsNavWrapper(env)
        self.face_env = McsFaceWrapper(env)
        #self.obj_env = McsObjWrapper(env)
        self.obstacles = {}
        #self.nav = BoundingBoxNavigator(robot_radius=AGENT_RADIUS)
        #if isinstance(self.nav, BoundingBoxNavigator):
        #self.env.add_obstacle_func = self.nav.add_obstacle_from_step_output
        #self.face = FaceTurnerResNet(get_action_space_from_names(self.face_env.action_names))
        self.sequence_generator_object = sequence_generator.SequenceGenerator(None, self.env.controller,level)

    def step(self, action_dict, epsd_collector=None, frame_collector=None,data_collection=False):
        assert 'action' in action_dict
        if action_dict['action'] == "GotoLocation":
            goal = get_goal(action_dict['location'])
            current_object_id = None

            '''
            for objectId,object_data in self.plannerState.object_loc_info.items():
                #object_data == goal:
                flag = 0
                for k in range(len(goal)):
                    if round(object_data[k],2) != goal[k]:
                        #print (round(object_data[k],2), goal[k])
                        flag = 1
                        break
                if flag == 0 :
                    current_object_id = objectId
                    break

            current_object_loc = self.plannerState.object_loc_info[current_object_id]
            if len(current_object_loc) == 3 :
                final_goal = goal[:]
                success_distance = machine_common_sense.mcs_controller_ai2thor.MAX_REACH_DISTANCE - 0.4
            else :
                final_goal = (float(current_object_loc[3]), float(current_object_loc[4]), float(current_object_loc[5]))
                print("Made a new goal {}".format(final_goal))
                success_distance = 0
            '''
            success_distance = 0.2
            print ("Goal from pddl stream", goal)

            success = self.sequence_generator_object.agent.nav.go_to_goal(
                #self.nav_env, goal, success_distance)
                goal, self.sequence_generator_object.agent,success_distance)
            if not success:
                print("Navigation Fail")
                return False
            #self.plannerState.agent_loc_info[self.plannerState.AGENT_NAME] = goal
            #self.plannerState.object_facing = None

        elif action_dict['action'] == "OpenObject" :
            print ("open object step")
            #action = "OpenObject,objectId="+ action_dict["objectId"]
            self.sequence_generator_object.agent.game_state.step(action_dict)
            if data_collection == True :
                action_dict_close = {"action": "CloseObject", "objectId": str(action_dict['objectId'])}
                self.sequence_generator_object.agent.game_state.step(action_dict_close)

        elif action_dict['action'] == "PickupObject" :
            print ("pick object step")
            #action = "PickupObject,objectId="+ action_dict["objectId"]
            #action = {"action", "PickupObject", ObjectId: "" }
            lookdown_action  = {"action":"LookDown"}
            lookup_action  = {"action":"LookUp"}
            self.sequence_generator_object.agent.game_state.step(lookdown_action)
            self.sequence_generator_object.agent.game_state.step(lookdown_action)
            self.sequence_generator_object.agent.game_state.step(action_dict)
            self.sequence_generator_object.agent.game_state.step(lookup_action)
            self.sequence_generator_object.agent.game_state.step(lookup_action)

        elif action_dict['action'] == "DropObject" :
            print ("Drop object step",action_dict)
            lookdown_action  = {"action":"LookDown"}
            lookup_action  = {"action":"LookUp"}
            self.sequence_generator_object.agent.game_state.step(lookdown_action)
            self.sequence_generator_object.agent.game_state.step(action_dict)
            self.sequence_generator_object.agent.game_state.step(lookup_action)

        elif action_dict['action'] == "MoveBack":
            self.sequence_generator_object.agent.game_state.step(action_dict)

        elif action_dict['action'] == "RotateLook":
            current_rotation = self.sequence_generator_object.agent.game_state.step_output.__dict__['rotation']
            theta = -(current_rotation - action_dict["final_angle"])
            print ("required rotation ", theta)
            print ("curr roation ", current_rotation)
            print ("final rot", action_dict["final_angle"])
            n = int(abs(theta) // 10)
            #ic (n)
            #if n <= 18 :
            if theta > 0:
                #action = {'action': 'RotateRight'}
                action = {'action': 'RotateLeft'}
                for _ in range(n):
                    self.sequence_generator_object.agent.game_state.step(action)
            else:
                action = {'action': 'RotateRight'}
                #action = {'action': 'RotateLeft'}
                for _ in range(n):
                    self.sequence_generator_object.agent.game_state.step(action)
            ic ("rotation after rotate actions: ",self.sequence_generator_object.agent.game_state.step_output.__dict__['rotation'] )
            #print ("req rot", required_rotation)
            #action = "RotateLook"
            #self.env.step(action="RotateLook", rotation=required_rotation)

        elif action_dict['action'] == "FaceToFront":
            FaceTurnerResNet.look_to_front(self.face_env)
            self.plannerState.face_to_front = True
            self.plannerState.object_facing = None
        elif action_dict['action'] == "FaceToObject":
            goal = get_goal(action_dict['location'])
            FaceTurnerResNet.look_to_direction(self.face_env, goal, epsd_collector)
            object_in_view = [PlanParser.create_legal_object_name(obj.uuid) for obj in self.env.step_output.object_list if not obj.held]
            if action_dict['objectId'] in object_in_view:
                self.plannerState.object_facing = action_dict['objectId']
            else:
                del self.plannerState.object_loc_info[action_dict['objectId']]
                for k in self.plannerState.object_containment_info:
                    if action_dict['objectId'] in self.plannerState.object_containment_info[k]:
                        self.plannerState.object_containment_info[k].remove(action_dict['objectId'])
                if action_dict['objectId'] in self.plannerState.object_open_close_info:
                    del self.plannerState.object_open_close_info[action_dict['objectId']]
                print("Object {} not at {}".format(action_dict['objectId'], action_dict['location']))
            self.plannerState.face_to_front = False
        elif action_dict['action'] == "PickupObject":
            self.obj_env.step("PickupObject", object_id=action_dict['objectId'], epsd_collector=epsd_collector)
            if self.env.step_output.return_status == "SUCCESSFUL":
                self.plannerState.object_in_hand = action_dict['objectId']
            else:
                print("Pickup {} fail!".format(action_dict['objectId']))
                return False


    def excecute(self):
        scene_config = main.explore_scene(self.sequence_generator_object, self.env.step_output)#'retrieval-', '0001'
        return True
