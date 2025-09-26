#!/usr/bin/env python3

import rospy
import tf.transformations as tft
import numpy as np
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from rosgraph_msgs.msg import Clock

class PayloadManager:
    def __init__(self):
        rospy.init_node('payload_manager')

        # Параметры
        self.model_name = rospy.get_param("~model_name", "Phone")
        self.model_file = rospy.get_param(
            "~model_file",
            "../models/Phone/model.sdf"
        )

        raw_offset = rospy.get_param("~offset", "[0.0, 0.18, 0.25]")
        if isinstance(raw_offset, str):
            try:
                raw_offset = raw_offset[1:-1]
                self.offset =  [float(x) for x in raw_offset.split(", ")]
            except Exception as e:
                rospy.logwarn("Failed to parse offset parameter, using default: %s", e)
                self.offset = [0.0, 0.18, 0.25]
        else:
            self.offset = raw_offset

        # Убедимся, что это список из 3 чисел
        if not isinstance(self.offset, (list, tuple)) or len(self.offset) != 3:
            rospy.logerr("Invalid offset parameter, must be list of 3 numbers. Got: %s", self.offset)
            self.offset = [0.0, 0.18, 0.25]

        self.model_file = os.path.expanduser(self.model_file)

        if not os.path.isfile(self.model_file):
            rospy.logfatal("Model file not found: %s", self.model_file)
            raise SystemExit(1)

        # Ожидание Gazebo
        rospy.loginfo("Waiting for /clock...")
        rospy.wait_for_message('/clock', Clock, timeout=30)

        # Сервисы Gazebo
        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=15)
        rospy.wait_for_service('/gazebo/delete_model', timeout=5)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5)
        rospy.wait_for_service('/link_attacher_node/attach', timeout=10)

        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        # Первый запуск: спавним payload
        self.first_spawn()

        # ROS-сервисы
        self.release_service = rospy.Service('/release_load', Trigger, self.handle_release)
        self.reset_service = rospy.Service('/reset_delivery', Trigger, self.handle_reset)

        rospy.loginfo("Payload manager ready. Model: '%s'", self.model_name)

    def get_drone_pose_gazebo(self):
        try:
            resp = self.get_model_state(model_name="clover", relative_entity_name="world")
            return resp.pose if resp.success else None
        except rospy.ServiceException as e:
            rospy.logerr("Gazebo get_model_state failed: %s", e)
            return None

    def transform_offset_gazebo(self, drone_pose, offset):
        q = drone_pose.orientation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]  # 3x3 матрица поворота
        offset_vec = np.array(offset)
        world_offset = R @ offset_vec

        new_pos = Point(
            x=drone_pose.position.x + world_offset[0],
            y=drone_pose.position.y + world_offset[1],
            z=drone_pose.position.z + world_offset[2]
        )
        return Pose(position=new_pos, orientation=drone_pose.orientation)

    def first_spawn(self):
        drone_pose = None
        for _ in range(30):
            drone_pose = self.get_drone_pose_gazebo()
            if drone_pose and (abs(drone_pose.position.x) > 0.01 or abs(drone_pose.position.y) > 0.01):
                break
            rospy.sleep(0.2)

        if drone_pose is None:
            rospy.logwarn("TF not ready, using fallback pose (0, 0.18, 0.2)")
            target_pose = Pose(
                position=Point(0.0, 0.18, 0.2),
                orientation=Quaternion(0, 0, 0, 1)
            )
        else:
            target_pose = self.transform_offset_gazebo(drone_pose, self.offset)

        # Спавним
        with open(self.model_file, 'r') as f:
            sdf = f.read()

        self.spawn_srv(
            model_name=self.model_name,
            model_xml=sdf,
            robot_namespace='',
            initial_pose=target_pose,
            reference_frame='world'
        )
        rospy.loginfo("Payload spawned.")

        # Прикрепляем
        rospy.sleep(0.3)
        req = AttachRequest()
        req.model_name_1 = "clover"
        req.link_name_1 = "base_link"
        req.model_name_2 = self.model_name
        req.link_name_2 = "link"
        self.attach_srv(req)
        rospy.loginfo("Payload attached.")

    def handle_release(self, req):
        try:
            req = AttachRequest()
            req.model_name_1 = "clover"
            req.link_name_1 = "base_link"
            req.model_name_2 = self.model_name
            req.link_name_2 = "link"
            self.detach_srv(req)
            return TriggerResponse(success=True, message="Payload released")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_reset(self, req):
        rospy.loginfo("Resetting delivery...")

        # Отсоединяем
        self.handle_release(None)

        # Получаем позу дрона
        drone_pose = None
        for _ in range(20):
            drone_pose = self.get_drone_pose_gazebo()
            if drone_pose:
                break
            rospy.sleep(0.1)

        if drone_pose is None:
            rospy.logwarn("Using fallback pose for reset")
            target_pose = Pose(position=Point(0.0, 0.18, 0.2), orientation=Quaternion(0,0,0,1))
        else:
            target_pose = self.transform_offset_gazebo(drone_pose, self.offset)

        # Перемещаем
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose = target_pose
        model_state.reference_frame = "world"
        self.set_model_state(model_state)

        # Прикрепляем
        rospy.sleep(0.2)
        req = AttachRequest()
        req.model_name_1 = "clover"
        req.link_name_1 = "base_link"
        req.model_name_2 = self.model_name
        req.link_name_2 = "link"
        self.attach_srv(req)

        return TriggerResponse(success=True, message="Delivery reset complete")

if __name__ == '__main__':
    try:
        PayloadManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass