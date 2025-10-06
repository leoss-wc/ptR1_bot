#!/usr/bin/env python3
#map_manager_node.py
import rospy
import os
import base64
import subprocess
import yaml
import shutil

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


from ptR1_navigation.srv import (ListMaps, ListMapsResponse, LoadMap, LoadMapResponse,
                                 GetMapFile, GetMapFileResponse, SaveMap, SaveMapResponse,
                                 StartSLAM, StartSLAMResponse, StopSLAM, StopSLAMResponse,
                                 DeleteMap, DeleteMapResponse, ResetSLAM, ResetSLAMResponse,
                                 StartPatrol, StartPatrolResponse, PausePatrol, PausePatrolResponse,
                                 ResumePatrol, ResumePatrolResponse, StopPatrol, StopPatrolResponse)


#รวมการจัดการ Process ไว้ที่นี่ที่เดียว
running_processes = []
navigation_process = None

MAP_FOLDER = os.path.expanduser('~/ptR1_ws/src/ptR1_navigation/maps')
ACTIVE_MAP_NAME = "active_map"


# ----------------- [LIST MAPS] -----------------
def handle_list_maps(req):
    rospy.loginfo("Listing maps...")
    names = []
    for file in os.listdir(MAP_FOLDER):
        if file.endswith(".yaml"):
            name = os.path.splitext(file)[0]
            if name != ACTIVE_MAP_NAME:
                names.append(name)
    return ListMapsResponse(names)

# ----------------- [LOAD MAP] ------------------
def handle_load_map(req):
    global navigation_process, running_processes
    map_to_load = req.name
    rospy.loginfo(f"Loading map '{map_to_load}' and starting mapserver, AMCL nodes...")

    # --- หยุด Process เก่า (ถ้ามี) ---
    # ใช้ฟังก์ชัน stop ที่มีอยู่แล้วเพื่อความสะอาด
    if navigation_process:
        rospy.loginfo("Stopping existing navigation process...")
        navigation_process.terminate()
        navigation_process.wait()
        if navigation_process in running_processes:
            running_processes.remove(navigation_process)
        navigation_process = None

    # --- ตรวจสอบว่าไฟล์แผนที่มีจริง ---
    map_yaml_path = os.path.join(MAP_FOLDER, f"{map_to_load}.yaml")
    if not os.path.exists(map_yaml_path):
        message = f"Map '{map_to_load}' not found."
        rospy.logerr(message)
        return LoadMapResponse(False, message)

    try:
        # --- สร้างคำสั่ง roslaunch ---
        # ส่ง map_name เข้าไปเป็น argument
        command = [
            'roslaunch',
            'ptR1_navigation',
            'navigation_2.launch',
            f'map_name:={map_to_load}'
        ]

        # --- รัน launch file ---
        rospy.loginfo(f"🚀 Executing: {' '.join(command)}")
        navigation_process = subprocess.Popen(command)
        running_processes.append(navigation_process)

        message = f"Navigation started with map '{map_to_load}'."
        rospy.loginfo(message)
        return LoadMapResponse(True, message)

    except Exception as e:
        message = f"Error starting navigation for map '{map_to_load}': {str(e)}"
        rospy.logerr(message)
        return LoadMapResponse(False, message)


# -------------- [GET MAP FILE] -----------------
def handle_get_map_file(req):
    rospy.loginfo(f"Getting map file: {req.name}")
    map_name = req.name
    image_path = os.path.join(MAP_FOLDER, f"{map_name}.png")
    yaml_path = os.path.join(MAP_FOLDER, f"{map_name}.yaml")

    if not os.path.exists(image_path):
        rospy.logwarn(f"[get_map_file] Not found: {image_path}")
        return GetMapFileResponse(
            success=False,
            message=f"Map image {map_name}.png not found",
            image_data_base64="",
            yaml_data=""
        )

    try:
        with open(image_path, 'rb') as f:
            encoded_image = base64.b64encode(f.read()).decode('utf-8')
        
        with open(yaml_path, 'r') as f:
            yaml_content = f.read()

        return GetMapFileResponse(
            success=True,
            message=f"{map_name}.png loaded",
            image_data_base64=encoded_image,
            yaml_data=yaml_content
        )
    except Exception as e:
        return GetMapFileResponse(
            success=False,
            message=str(e),
            image_data_base64="",
            yaml_data=""
        )

def handle_delete_map(req):
    map_name = req.name
    rospy.loginfo(f"Received request to delete map: {map_name}")

    # สร้าง list ของ path ไฟล์ทั้งหมดที่เกี่ยวข้องกับแผนที่นี้
    files_to_check = [
        os.path.join(MAP_FOLDER, f"{map_name}.yaml"),
        os.path.join(MAP_FOLDER, f"{map_name}.pgm"),
        os.path.join(MAP_FOLDER, f"{map_name}.png")
    ]
    
    deleted_count = 0
    try:
        # วนลูปเพื่อลบไฟล์แต่ละไฟล์
        for file_path in files_to_check:
            if os.path.exists(file_path):
                os.remove(file_path)
                rospy.loginfo(f"  - Deleted: {os.path.basename(file_path)}")
                deleted_count += 1
        
        if deleted_count > 0:
            message = f"Map '{map_name}' and associated files were deleted."
            rospy.loginfo(message)
            return DeleteMapResponse(success=True, message=message)
        else:
            message = f"Map '{map_name}' not found. Nothing to delete."
            rospy.logwarn(message)
            return DeleteMapResponse(success=True, message=message)

    except OSError as e:
        message = f"Error deleting map '{map_name}': {str(e)}"
        rospy.logerr(message)
        return DeleteMapResponse(success=False, message=message)

# ----------------- [SAVE MAP] -----------------
def handle_save_map(req):
    name = req.name
    pgm_path = os.path.join(MAP_FOLDER, f"{name}.pgm")
    png_path = os.path.join(MAP_FOLDER, f"{name}.png")

    rospy.loginfo(f"Saving map to {name}")

    try:
        subprocess.check_call([
            'rosrun', 'map_server', 'map_saver',
            '-f', os.path.join(MAP_FOLDER, name),
            'map:=/rb/slam/map'
        ])
        subprocess.check_call([
            'convert', pgm_path, png_path
        ])
        
        return SaveMapResponse(
            success=True,     
            message=f"Map saved and converted to {png_path}"
        )
    except subprocess.CalledProcessError as e:
        return SaveMapResponse(
            success=False,
            message=str(e)
        )

# ----------------- [START SLAM] -----------------
def handle_start_slam(req):
    global running_processes

    handle_stop_slam(None) 
    
    rospy.loginfo("Starting SLAM using ptR1_navigation/slam.launch...")

    try:
        # 2. สร้างคำสั่งสำหรับรัน launch file
        slam_launch_command = [
            'roslaunch',
            'ptR1_navigation',
            'slam.launch'
        ]

        # 3. รันคำสั่ง roslaunch ผ่าน subprocess.Popen
        process = subprocess.Popen(slam_launch_command)
        
        # 4. เพิ่ม process ที่รันใหม่เข้าไปใน list เพื่อให้ stop_slam จัดการได้
        running_processes.append(process)
        
        rospy.loginfo("Started SLAM process from launch file.")
        return StartSLAMResponse(success=True, message="SLAM started successfully via launch file.")

    except Exception as e:
        rospy.logerr(f"Failed to start SLAM launch file: {e}")
        # หากเกิดข้อผิดพลาด ให้พยายามหยุด process ที่อาจจะค้างอยู่
        handle_stop_slam(None)
        return StartSLAMResponse(success=False, message=str(e))
       
# ----------------- [STOP SLAM] -----------------
def handle_stop_slam(req):
    global running_processes

    if not running_processes:
        if req is not None:
             rospy.loginfo("No managed processes were running.")
        return StopSLAMResponse(success=True, message="No managed processes were running.")

    rospy.loginfo(f"Stopping {len(running_processes)} managed processes...")
    
    try:
        for process in running_processes:
            process.terminate()
            process.wait(timeout=5)
        
        running_processes = []
        rospy.loginfo("All managed processes terminated.")
        return StopSLAMResponse(success=True, message="All managed processes terminated successfully.")
    
    except subprocess.TimeoutExpired:
        rospy.logerr("Timeout expired while waiting for a process to terminate. Forcing kill.")
        for process in running_processes:
            process.kill()
        running_processes = []
        return StopSLAMResponse(success=False, message="Timeout expired, processes were killed.")

    except Exception as e:
        rospy.logerr(f"Failed to stop SLAM processes: {str(e)}")
        return StopSLAMResponse(success=False, message=f"Failed to stop SLAM: {str(e)}")

def handle_reset_slam(req):
    rospy.loginfo("Received request to reset SLAM.")
    
    # ชื่อ service ของ slam_toolbox
    slam_reset_service_name = '/slam_toolbox/reset'
    
    try:
        # รอให้ service ของ slam_toolbox พร้อมใช้งาน
        rospy.wait_for_service(slam_reset_service_name, timeout=2.0)
        
        # สร้างตัวเรียก service
        reset_slam_service = rospy.ServiceProxy(slam_reset_service_name, Empty)
        
        # เรียก service
        reset_slam_service()
        
        message = "SLAM has been successfully reset."
        rospy.loginfo(message)
        return ResetSLAMResponse(success=True, message=message)
        
    except rospy.ROSException as e:
        message = f"Service '{slam_reset_service_name}' is not available. Is SLAM running?"
        rospy.logerr(message)
        return ResetSLAMResponse(success=False, message=message)
        
    except rospy.ServiceException as e:
        message = f"Failed to call service '{slam_reset_service_name}': {e}"
        rospy.logerr(message)
        return ResetSLAMResponse(success=False, message=message)

# ----------------- [CLEAR COSTMAPS] -----------------
def handle_clear_costmaps(req):
    rospy.loginfo("Received request to clear costmaps.")
    service_name = '/move_base/clear_costmaps'
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
        clear_costmaps_service = rospy.ServiceProxy(service_name, Empty)
        clear_costmaps_service()
        message = "Costmaps cleared successfully."
        rospy.loginfo(message)
        return ClearCostmapsResponse(success=True, message=message)
    except Exception as e:
        message = f"Failed to clear costmaps: {e}"
        rospy.logerr(message)
        return ClearCostmapsResponse(success=False, message=message)



# -------------- [SHUTDOWN HOOK] -----------------
def shutdown_hook():
    rospy.loginfo("Shutdown request received...")
    if running_processes:
        handle_stop_slam(None)
    rospy.loginfo("Goodbye!")

# ----------------- [MAIN NODE] -----------------
def map_manager_server():
    rospy.init_node('map_manager')

    rospy.Service('/map_manager/list_maps', ListMaps, handle_list_maps)
    rospy.Service('/map_manager/load_map', LoadMap, handle_load_map)
    rospy.Service('/map_manager/get_map_file', GetMapFile, handle_get_map_file)
    rospy.Service('/map_manager/save_map', SaveMap, handle_save_map)
    rospy.Service('/map_manager/start_slam', StartSLAM, handle_start_slam)
    rospy.Service('/map_manager/stop_slam', StopSLAM, handle_stop_slam) 
    rospy.Service('/map_manager/delete_map', DeleteMap, handle_delete_map)
    rospy.Service('/map_manager/reset_slam', ResetSLAM, handle_reset_slam)

    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("Map Manager Services Ready")
    rospy.spin()

if __name__ == '__main__':
    map_manager_server()