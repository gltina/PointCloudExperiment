from ast import While
import threading
import numpy as np
import copy
import open3d as o3d
import time
from threading import Thread, Timer

picked_points = []

count = 0
vis_flag = False


def thread_pick_points_from_pointcloud(pc):
    global vis_flag
    print(f"1My threading ID is {threading.get_ident()}")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.add_geometry(pc)
    vis.create_window()
    vis.run()
    vis.destroy_window()
    vis_flag = True


def thread_collect_picked_points(args):
    f = open("demofile3.txt", "a+")
    print(f"2My threading ID is {threading.get_ident()}")
    global count
    global vis_flag
    while vis_flag == False:
        count += 1
        # picked_points = vis.get_picked_points()
        # f.write(picked_points)
        f.write(f"{count}:collected the picked points.\n")
        print("test")
        time.sleep(0.5)
    f.close()


if __name__ == "__main__":
    print(f"3My threading ID is {threading.get_ident()}")
    pcd = o3d.io.read_point_cloud("./test_points.pcd")
    th_point_collect = Thread(
        target=thread_collect_picked_points, args=(1,))
    th_point_collect.start()
    thread_pick_points_from_pointcloud(pcd)
