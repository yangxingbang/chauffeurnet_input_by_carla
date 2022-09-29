"""
In this script, we are going to learn how to spawn a vehicle on the road and make it autopilot.
At the same time, we will collect camera and lidar data from it.
"""

import carla
import os
import random
import cv2


def main():
    actor_list = []
    sensor_list = []

    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        # print(client.get_available_maps())

        # you can also retrive another world by specifically defining
        world = client.load_world('Town02_Opt', map_layers=carla.MapLayer.NONE)
        map_none = world.get_map()
        
        blueprint_library = world.get_blueprint_library()
        # create the ego vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.volkswagen.t2')
        # 自车白色
        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        # get a random valid occupation in the world
        transform = random.choice(world.get_map().get_spawn_points())
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        ego_vehicle.set_autopilot(True)

        # Nearest waypoint in the center of a Driving or Sidewalk lane.
        waypoint01 = map_none.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

        '''
        以下打印都只能输出一帧的数据，怎么每帧都输出？
        '''

        #Nearest waypoint but specifying OpenDRIVE parameters. 
        waypoint02 = map_none.get_waypoint_xodr(1,1,10.0)
        print('waypoint02: ', waypoint02)
        waypoint_list = map_none.generate_waypoints(20.0)
        print('waypoint_list: ', waypoint_list)
        waypoint_tuple_list = map_none.get_topology()
        print('waypoint_tuple_list: ', waypoint_tuple_list)
        print('ego_acceleration: ', ego_vehicle.get_acceleration())
        print('ego_velocity: ',ego_vehicle.get_velocity())
        # 自车的bounding box，以及用box取信息
        ego_box = ego_vehicle.bounding_box
        print(ego_box.location)
        print(ego_box.extent)  

        # 自车添加到actor列表
        actor_list.append(ego_vehicle)

        # 添加俯视摄像机，他能牌照一个80m*80m的区域，并且能随着车辆移动
        camera_rgb = blueprint_library.find('sensor.camera.rgb')
        camera_location = carla.Location(x=30, z=50)
        camera_rotation = carla.Rotation(pitch=-90)
        camera_transform = carla.Transform(camera_location, camera_rotation)
        print(camera_rgb.get_attribute('image_size_x'))
        print(camera_rgb.get_attribute('image_size_y'))
        # 俯视相机输出图片400 * 400像素
        camera_rgb.set_attribute('image_size_x', '400')
        camera_rgb.set_attribute('image_size_y', '400')
        # 俯视相机输出图片0.2s拍一次，我将此作为5帧
        camera_rgb.set_attribute('sensor_tick', '0.2')
        print(camera_rgb.get_attribute('sensor_tick'))
        camera = world.spawn_actor(camera_rgb, camera_transform, attach_to=ego_vehicle)

        # set the callback function
        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(lambda image: image.save_to_disk('output_path/%06d.png' % image.frame, cc))
        sensor_list.append(camera)


        while True:
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))

    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')

