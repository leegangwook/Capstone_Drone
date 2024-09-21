import json
import websockets
import asyncio
import threading
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import VelocityBodyYawspeed
import firebase_admin
from firebase_admin import credentials, firestore

async def connect(drone):
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
       
    return drone
       
async def send_data(drone, server_uri, logger):
    print('sned data start')
    print(drone)
    async with websockets.connect(server_uri) as websocket:
        try:
             # 클라이언트가 서버에 연결됨을 알리는 메시지 전송
            await websocket.send("streamer")
           
            while True:
                async for position in drone.telemetry.position():
                    latitude = position.latitude_deg
                    longitude = position.longitude_deg
                    altitude = position.relative_altitude_m
                    break

                async for attitude_info in drone.telemetry.attitude_euler():
                    yaw = attitude_info.yaw_deg
                    pitch = attitude_info.pitch_deg
                    roll = attitude_info.roll_deg
                    break

                '''async for battery in drone.telemetry.battery():
                    remaining_battery= battery.remaining_percent
                    break'''
       
                data = {
                    'latitude': latitude,
                    'longitude': longitude,
                    'altitude': altitude,
                    'yaw': yaw,
                    'pitch': pitch,
                    'roll': roll,
                    #'battery': remaining_battery
                }
                print(data)

                await websocket.send(json.dumps(data))
                await asyncio.sleep(0.2)

        except Exception as e:
            print("error:", e)
            logger.add_log('send_data', e)
           
class DroneController:
    def __init__(self, drone, logger):
        self.drone = drone
        self.logger = logger

    async def arm(self):
        try:
            await self.drone.action.arm()
        except Exception as e:
            self.logger.add_log('arm', e)
        await asyncio.sleep(3)

    async def disarm(self):
        try:
            await self.drone.action.disarm()
        except Exception as e:
            self.logger.add_log('disarm', e)
        await asyncio.sleep(2)

    async def takeoff(self):
        try:
            await self.drone.action.takeoff()
        except Exception as e:
            self.logger.add_log('takeoff', e)
        await asyncio.sleep(10)
   
    async def land(self):
        await self.drone.action.land()
        try:
            await self.drone.action.disarm()
        except Exception as e:
            self.logger.add_log('land', e)
        await asyncio.sleep(10)
       
    async def hold(self):
        await self.drone.action.hold()
        try:
            await self.drone.action.disarm()
        except Exception as e:
            self.logger.add_log('hold', e)

    async def upload_mission(self, mission_plan, meter):
        try:
            await self.drone.mission.set_return_to_launch_after_mission(True)
            await self.drone.action.set_return_to_launch_altitude(meter)
            await self.drone.mission.upload_mission(mission_plan)
            print(f"Mission {len(mission_plan.mission_items)} added!")
            # return len(mission_plan.mission_items)
        except Exception as e:
            self.logger.add_log('upload_mission', e)

    async def start_mission(self):
        try:
            await self.drone.mission.start_mission()
        except Exception as e:
            self.logger.add_log('start_mission', e)

    async def pause_mission(self):
        try:
            await self.drone.mission.pause_mission()
        except Exception as e:
            self.logger.add_log('pause_mission', e)

    async def resume_mission(self):
        try:
            await self.drone.mission.start_mission()
        except Exception as e:
            self.logger.add_log('resume_mission', e)
   
    async def finished_mission(self):
        try:
            return await self.drone.mission.is_mission_finished()
        except Exception as e:
            self.logger.add_log('finished_mission', e)
   
    #async def clear_mission(self):
        #await self.drone.mission.clear_mission()
       
    async def manual_forward(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(velocity, 0.0, 0.0, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_forward', e)
        #await self.drone.offboard.stop()
       
    async def manual_back(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(-velocity, 0.0, 0.0, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_back', e)
        #await self.drone.offboard.stop()
       
    async def manual_right(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, velocity, 0.0, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_right', e)
        #await self.drone.offboard.stop()
       
    async def manual_left(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -velocity, 0.0, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_left', e)
        #await self.drone.offboard.stop()
       
    async def manual_up(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -velocity, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_up', e)
        #await self.drone.offboard.stop()
       
    async def manual_down(self, meter, velocity):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, velocity, 0.0))
            await asyncio.sleep(meter)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_down', e)
        #await self.drone.offboard.stop()
       
    async def manual_turn_right(self, deg):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, deg/5))
            await asyncio.sleep(5.458)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_turn_right', e)
        #await self.drone.offboard.stop()
   
    async def manual_turn_left(self, deg):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, -deg/5))
            await asyncio.sleep(5.458)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('manual_turn_left', e)
        #await self.drone.offboard.stop()
     
    async def a_move_right_60(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.732, 1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_right_60', e)
        #await self.drone.offboard.stop()
   
    async def a_move_right_45(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_right_45', e)
        #await self.drone.offboard.stop())
   
    async def a_move_right_30(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.57735, 1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_right_30', e)
        #await self.drone.offboard.stop()
   
    async def a_move_right_15(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.26795, 1.0, 0.0, 0.0))
            await asyncio.sleep(0.1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_right_15', e)
        #await self.drone.offboard.stop()
   
    async def a_move_right(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_right', e)
        #await self.drone.offboard.stop()
       
    async def a_move_left(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_left', e)
        #await self.drone.offboard.stop()
   
    async def a_move_left_15(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.26795, -1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_left_15', e)
        #await self.drone.offboard.stop()
       
    async def a_move_left_30(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.57735, -1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_left_30', e)
        #await self.drone.offboard.stop()
       
    async def a_move_left_45(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, -1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_left_45', e)
        #await self.drone.offboard.stop()
   
    async def a_move_left_60(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.732, -1.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_left_60', e)
        #await self.drone.offboard.stop()
   
    async def a_move_back(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1)
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.action.hold()
        except Exception as e:
            self.logger.add_log('a_move_back', e)
        #await self.drone.offboard.stop()

class MissionExecutor:
    def __init__(self, drone_controller, doc_ref_mission, doc_ref_mission2):
        self.drone_controller = drone_controller
        self.doc_ref_mission = doc_ref_mission
        self.doc_ref_mission2 = doc_ref_mission2
        self.drone_mission_finished = {"header":"finish"}
        self.latest_mission = None
        self.latest_deg = None
        self.latest_meter = None
        self.latest_velocity = None
        self.latest_alt = None
        self.latest_lat = None
        self.latest_lon = None
        self.mission_items = []
        self.mission_status = 'Waiting'

    async def execute_mission(self):
        while True:
            flag = await self.on_mission_snapshot()
            
            if self.mission_status == 'Performing':
                if await self.drone_controller.finished_mission():
                    await self.drone_controller.pause_mission()
                    print("--Mission finished")
                    self.doc_ref_mission.update(self.drone_mission_finished)
                    self.mission_status = 'Waiting'
                    
           
            if flag:
                if self.latest_mission == 'disarm':
                    print("--Disarm")
                    await self.drone_controller.disarm()
               
                elif self.latest_mission == 'arm':
                    print("--Arm")
                    await self.drone_controller.arm()

                elif self.latest_mission == 'takeoff':
                    print("--Takeoff")
                    await self.drone_controller.takeoff()

                elif self.latest_mission == 'land':
                    print("--Land")
                    await self.drone_controller.land()

                elif self.latest_mission == 'upload':
                    print("--Upload")
                    await self.create_mission_plan()

                elif self.latest_mission == 'mission':
                    meter = await self.on_meter_snapshot()
                    mission_plan = MissionPlan(self.mission_items)
                    await self.drone_controller.upload_mission(mission_plan, meter)
                    await asyncio.sleep(5)
                    print("--Start mission")
                    await self.drone_controller.arm()
                    await self.drone_controller.takeoff()
                    await self.drone_controller.start_mission()
                    
                    self.mission_status = 'Performing'
                   
                    await self.drone_controller.pause_mission()
                    print("--Mission finished")
                    self.doc_ref_mission.update(self.drone_mission_finished)
                   
                elif self.latest_mission == 'pause':
                    print("--Pause mission")
                    await self.drone_controller.pause_mission()
               
                elif self.latest_mission == 'resume':
                    print("--Resume mission")
                    await self.drone_controller.resume_mission()
                   
                elif self.latest_mission == 'return':
                    print("--Return to launch after mission")
                    await self.drone_controller.resume_mission()
                   
                #elif self.latest_mission == 'clear':
                    #print("--Clear mission item")
                    #await self.drone_controller.clear_mission()
                   
                elif self.latest_mission == 'manual_forward':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <forward %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_forward(meter, velocity)
                   
                elif self.latest_mission == 'manual_back':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <back %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_back(meter, velocity)
                   
                elif self.latest_mission == 'manual_right':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <right %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_right(meter, velocity)
                   
                elif self.latest_mission == 'manual_left':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <left %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_left(meter, velocity)
                   
                elif self.latest_mission == 'manual_up':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <up %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_up(meter, velocity)
                   
                elif self.latest_mission == 'manual_down':
                    meter = await self.on_meter_snapshot()
                    velocity = await self.on_velocity_snapshot()
                    print("--Manual <down %.1f meter>"%(meter*velocity))
                    await self.drone_controller.manual_down(meter, velocity)
                       
                elif self.latest_mission == 'manual_turn_right':
                    deg = await self.on_degree_snapshot()
                    print("--Manual <right %.1f degree>"%(deg))
                    await self.drone_controller.manual_turn_right(deg)
                   
                elif self.latest_mission == 'manual_turn_left':
                    deg = await self.on_degree_snapshot()
                    print("--Manual <left %.1f degree>"%(deg))
                    await self.drone_controller.manual_turn_left(deg)
                   
                elif self.latest_mission == 'avoid_start':
                    print("--Obstacle detection. Starting collision avoidance")
                    await self.drone_controller.pause_mission()
                   
                elif self.latest_mission == 'avoid_finish':
                    print("--Collision avoidance complete.")
                    await self.drone_controller.hold()
                   
                elif self.latest_mission == 'right_60':
                    print("--Move to <right 60 degree>")
                    await self.drone_controller.a_move_right_60()
               
                elif self.latest_mission == 'right_45':
                    print("--Move to <right 45 degree>")
                    await self.drone_controller.a_move_right_45()
               
                elif self.latest_mission == 'right_30':
                    print("--Move to <right 30 degree>")
                    await self.drone_controller.a_move_right_30()
               
                elif self.latest_mission == 'right_15':
                    print("--Move to <right 15 degree>")
                    await self.drone_controller.a_move_right_15()
               
                elif self.latest_mission == 'right':
                    print("--Move to <right>")
                    await self.drone_controller.a_move_right()
               
                elif self.latest_mission == 'left':
                    print("--Move to <left>")
                    await self.drone_controller.a_move_left()
                   
                elif self.latest_mission == 'left_15':
                    print("--Move to <left 15 degree>")
                    await self.drone_controller.a_move_left_15()
               
                elif self.latest_mission == 'left_30':
                    print("--Move to <left 30 degree>")
                    await self.drone_controller.a_move_left_30()
                   
                elif self.latest_mission == 'left_45':
                    print("--Move to <left 45 degree>")
                    await self.drone_controller.a_move_left_45()
                   
                elif self.latest_mission == 'left_60':
                    print("--Move to <left 60 degree>")
                    await self.drone_controller.a_move_left_60()
               
                elif self.latest_mission == 'back':
                    print("--Move to <back>")
                    await self.drone_controller.a_move_back()
                   
                elif self.latest_mission == 'emergency':
                    print("--Emergency!!!")
                    await self.drone_controller.pause_mission()
                    await self.drone_controller.land()
                else:
                    print("--Latest Mission Invalid. Latest mission : ", self.latest_mission)
            else:
                await asyncio.sleep(1)
    
    async def on_mission_snapshot(self):
        mission = self.doc_ref_mission.get().to_dict().get('header')
           
        if mission != self.latest_mission:
            self.latest_mission = mission
            return True

        return False
   
    async def create_mission_plan(self):
        mission_data = self.doc_ref_mission2.get().to_dict()
       
        if mission_data:
            lat_str = mission_data.get('lat')
            lon_str = mission_data.get('lon')
            alt_str = mission_data.get('alt')

            if lat_str is not None and lon_str is not None:
                lat = float(lat_str)
                lon = float(lon_str)
                alt = float(alt_str)

                if lat != self.latest_lat and lon != self.latest_lon:
                    new_mission_item = MissionItem(lat, lon, alt, 1, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE)
                    self.mission_items.append(new_mission_item)
                    print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")

                    self.latest_lat = lat
                    self.latest_lon = lon
                    self.latest_alt = alt
        else:
            print("Latitude or Longitude is None")
           
    async def on_degree_snapshot(self):
        degree_data = self.doc_ref_mission.get().to_dict()
           
        if degree_data:
            latest_deg = degree_data.get('degree')
            if latest_deg is not None:
                deg = float(latest_deg)
                self.latest_deg = deg
                return deg
                   
    async def on_meter_snapshot(self):
        meter_data = self.doc_ref_mission.get().to_dict()
           
        if meter_data:
            latest_meter = meter_data.get('meter')
            if latest_meter is not None:
                meter = float(latest_meter)
                self.latest_meter = meter
                return meter

    async def on_velocity_snapshot(self):
        velocity_data = self.doc_ref_mission.get().to_dict()
           
        if velocity_data:
            latest_velocity = velocity_data.get('velocity')
            if latest_velocity is not None:
                velocity = float(latest_velocity)
                self.latest_velocity = velocity
                return velocity


class Logger:
    def __init__(self):
        self.log_file_path = './log.txt'
        self.log_file = None
    
    def open_log_file(self):
        # 'a' 모드는 파일이 존재하지 않으면 생성하고, 존재하면 파일의 끝에 내용을 추가합니다.
        self.log_file = open(self.log_file_path, 'a')
    
    def add_log(self, function_name, content):
        if self.log_file is None:
            self.log_file = open(self.log_file_path, 'a')
        self.log_file.write(function_name + ':::' + content + '\n')
    
    def save_log(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        



async def main():
    logger = Logger()
    logger.open_log_file()
    # 드론 연결
    drone = System()
   
    # Firebase 및 Firestore 설정
    cred = credentials.Certificate("/home/wook/Desktop/CapDrone_key.json")
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    doc_ref_mission = db.collection("Capston").document("drone")
    doc_ref_mission2 = db.collection("Capston").document("map")

    # 변수 추가
    drone_condition_unconnected = {"condition":"unconnect"}
    drone_condition_connected = {"condition":"connect"}
    drone_header_reset = {"header":""}
    drone_degree_default = {"degree":"45"}
    drone_meter_default = {"meter":"5"}
    drone_velocity_default = {"velocity":"1"}
    #map_lat_reset = {"lat":""}
    #map_lon_reset = {"lon":""}
    map_lon_default = {"alt":"10"}

    # 초기 상태 설정
    doc_ref_mission.update(drone_condition_unconnected)
    doc_ref_mission.update(drone_header_reset)
    doc_ref_mission.update(drone_degree_default)
    doc_ref_mission.update(drone_meter_default)
    doc_ref_mission.update(drone_velocity_default)
    #doc_ref_mission2.update(map_lat_reset)
    #doc_ref_mission2.update(map_lon_reset)
    doc_ref_mission2.update(map_lon_default)
    print("Firestore connected!")
   
    # 드론 연결 및 미션 실행
    drone = await connect(drone)
    doc_ref_mission.update(drone_condition_connected)
   
    # 스트리머
    server_uri = "ws://203.255.57.136:5252"
   
    # 컨트롤러
    drone_controller = DroneController(drone, logger)
    mission_executor = MissionExecutor(drone_controller, doc_ref_mission, doc_ref_mission2)

    loop = asyncio.get_event_loop()
   
    streamer_task = loop.create_task(send_data(drone, server_uri, logger))
    mission_task = loop.create_task(mission_executor.execute_mission())
   
   
    try:
        # 스트림 전송 및 미션 실행을 동시에 실행
        await asyncio.gather(
            streamer_task,
            mission_task
        )
    except KeyboardInterrupt:
        pass
    finally:
        logger.save_log()
    

    # 종료 시 상태 초기화
    doc_ref_mission.update(drone_condition_unconnected)
    doc_ref_mission.update(drone_header_reset)
    print("Drone disconnected!")


if __name__ == "__main__":
    asyncio.run(main())