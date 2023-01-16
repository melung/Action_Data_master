from mlsocket import MLSocket
import socket
import numpy as np
from Skeleton_Fusion import Skeleton_Fusion
import multiprocessing as mp
from multiprocessing import shared_memory
import timeit
import time
import os
import keyboard
import tkinter
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



#HOST = '169.254.164.143'#Master 컴퓨터 ip
HOST = '192.168.0.5'#Master 컴퓨터 ip

#CLIENTS_LIST = ['169.254.164.144', '169.254.164.145','169.254.164.146'] #slave 컴퓨터 IP EX) 3개 컴퓨터 이용시 [A, B, C]
CLIENTS_LIST = ['192.168.0.5', '169.254.164.145','169.254.164.146'] #slave 컴퓨터 IP EX) 3개 컴퓨터 이용시 [A, B, C]


Port = [1144, 1145, 1146] #slave 컴퓨터마다 통신할 port 지정 EX) 3개 컴퓨터 이용시 [가, 나, 다]
start_port = 12345
end_port = 4444


num_com = 1 #slave 컴퓨터개수
num_source = 2 #한 slave 컴퓨터에 연결된 kinect 개수
fps_cons = 30 #fps제한 (컴퓨터에 따라 다를 수 있습니다. 적절히 조절 slave 컴퓨터의 kinect fps와 같도록)
vis = True


Vive_ip = '192.168.0.9'
Vive_port = 1234



action_list = ["Hands up","Forward hand","T-pose","Standing","Crouching","Open","Change grenade","Throw high","Throw low","Bend","Lean","Walking","Run","Shooting Pistol","Reload Pistol","Crouching Pistol","Change Pistol","Shooting Rifle","Reload Rifle","Crouching Rifle","Change Rifle","Holding high knife","Stabbing Knife","Change knife"]
s_idx = '0 22 23 24 0 18 19 20 0 1 2 3 2 11 12 13 14 15 14 2 4 5 6 7 8 7'
f_idx = '22 23 24 25 18 19 20 21 1 2 3 26 11 12 13 14 15 16 17 4 5 6 7 8 9 10'
s_idx = np.asarray(s_idx.split(), dtype=int)
f_idx = np.asarray(f_idx.split(), dtype=int)
cur_time = str(time.localtime().tm_year)+str(time.localtime().tm_mon) +str(time.localtime().tm_mday)+str(time.localtime().tm_min)


def get_data(HOST, PORT, CLIENTS_LIST, shm_nm, nptype, kk):
    temp_shm = shared_memory.SharedMemory(name=shm_nm)
    arr = np.frombuffer(buffer=temp_shm.buf, dtype=nptype)

    ADDR = (HOST, PORT + 100 * kk)
    serverSocket = MLSocket()
    serverSocket.bind(ADDR)

    while True:

        serverSocket.listen(0)
        clientSocket, addr_info = serverSocket.accept()

        # print(addr_info[0])
        data = clientSocket.recv(1024)
        if addr_info[0] == CLIENTS_LIST[0]:
            arr[:] = data
        elif addr_info[0] == CLIENTS_LIST[1]:
            arr[:] = data

def get_vive_data(HOST, PORT, CLIENTS_LIST, shm_nm, nptype):
    temp_shm = shared_memory.SharedMemory(name = shm_nm)
    arr = np.frombuffer(buffer=temp_shm.buf, dtype=nptype)

    ADDR = (HOST, PORT)

    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(ADDR)

    while True:
        serverSocket.listen(0)
        clientSocket, addr_info = serverSocket.accept()

        #print(addr_info[0])
        data = clientSocket.recv(1024)
        text = data.decode('utf-8')
        joint_array = np.array(text.split())
        joint_array = joint_array.astype(np.float)
        #print(joint_array)


        arr[:] = joint_array

if __name__ == '__main__':

    total_joints = np.zeros((num_source*num_com * 27 * 4,), dtype=float)

    for ii in range(num_com):
        for kk in range(num_source):
            locals()[f"com{ii}_{kk}_joints"] = np.zeros((27 * 4,), dtype=float)

    print(np.shape(locals()["com0_0_joints"]))

    for ii in range(num_com):
        for kk in range(num_source):
            locals()[f"shm{ii}_{kk}"] = shared_memory.SharedMemory(create=True, size=locals()[f"com{ii}_{kk}_joints"].nbytes)
            locals()[f"shared_joints{ii}_{kk}"] = np.frombuffer(locals()[f"shm{ii}_{kk}"].buf, locals()[f"com{ii}_{kk}_joints"].dtype)
            locals()[f"shared_joints{ii}_{kk}"][:] = locals()[f"com{ii}_{kk}_joints"]
    print(np.shape(locals()["shared_joints0_0"]))

    vive_joints = np.zeros((3 * 3,), dtype=float)
    shm_vive = shared_memory.SharedMemory(create=True, size=vive_joints.nbytes)
    vive_shared_joints = np.frombuffer(shm_vive.buf, vive_joints.dtype)
    vive_shared_joints [:] = vive_joints



    for ii in range(num_com):
        for kk in range(num_source):
            locals()[f"com{ii}_{kk}"] = mp.Process(target=get_data, args=[HOST, Port[ii], CLIENTS_LIST, locals()[f"shm{ii}_{kk}"].name,locals()[f"com{ii}_{kk}_joints"].dtype, kk])

    vive_com = mp.Process(target=get_vive_data, args=[HOST, Vive_port, CLIENTS_LIST, shm_vive.name,
                                                             vive_joints.dtype])

    for ii in range(num_com):
        for kk in range(num_source):
            locals()[f"com{ii}_{kk}"].daemon = True
            locals()[f"com{ii}_{kk}"].start()
    vive_com.daemon = True
    vive_com.start()

    b = input("Press Enter to start")

    for ii in range(num_com):
        for kk in range(num_source):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((CLIENTS_LIST[ii], start_port + kk * 100))
            s.sendto(str(b).encode(), (CLIENTS_LIST[ii], start_port + kk * 100))
            s.close()


    End = False
    total_joints = np.zeros((num_source * num_com * 27 * 4,), dtype=float)

    while True:
        for i in range(len(action_list)):
            print("Action" + format(i, "02") + " : " + action_list[i])
        while True:
            ss = input("Subject num or e to Stop: ")
            if ss == 'e':
                print('End')
                for ii in range(num_com):
                    for kk in range(num_source):
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.connect((CLIENTS_LIST[ii], end_port + 100*kk))
                        s.close()
                End = True
                break
            else:
                try:
                    subject = int(ss)
                except Exception:
                    print("That's not a valid option!")
            try:
                action = int(input("Select Action number : "))
                if action < len(action_list) and action >= 0:
                    break
                else:
                    print("Wrong Range")
            except Exception:
                print("That's not a valid option!")
        if End == True:
            break
        print('Save Data Subject : '+ str(subject) +' Action : '+ str(action))
        b = input("Press a to Save Data")
        if b == "a":
            print("Start")
        else:
            print("Reselect Action")
            continue

        Fused_skel = []

        f = open(
            "./data/S" + format(subject, "03") + "C" + format(0, "03") + "P000R000A" + format(action,
                                                                                               "03") + ".skeleton",
            'w')
        f.write("frame\n")

        for ii in range(num_com):
            for kk in range(num_source):
                if not os.path.exists(
                        "./Temp_Result/S" + format(subject, "03") + "C" + format(ii, "03") + "P"+ format(kk,"03") + "R000A" + format(action,
                                                                                                                 "03")):
                    os.makedirs(
                        "./Temp_Result/S" + format(subject, "03") + "C" + format(ii, "03") + "P"+ format(kk,"03") + "R000A" + format(action,
                                                                                                                 "03"))
        if vis:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)
            ax.set_zlim(-7, 7)
            ax.set_box_aspect((2, 2, 3))
        k = 0
        while True:
            if k == 0:
                time.sleep(0.05)
            start_t = timeit.default_timer()
            if k > 1:
                f.write(str(1) + "\n")  # numbody
                f.write("1 1 1 1 1 1 1 1 1 1\n")
                f.write(str(26) + "\n")  # num joint

            for ii in range(num_com):
                for kk in range(num_source):
                    locals()[f"com{ii}_{kk}_joints"][:] = locals()[f"shared_joints{ii}_{kk}"]
                    total_joints[27 * 4 * (2 * ii + kk):27 * 4 * (2 * ii + kk + 1)] = locals()[f"com{ii}_{kk}_joints"][
                                                                                      :]

            vive_joints[:] = vive_shared_joints
            print(vive_joints)

            #print(locals()[f"com{ii}_joints"][:])
            Fuse = Skeleton_Fusion(camnum=num_source*num_com, joint=total_joints,
                                                    pre_joint=[] if k == 0 else Fused_skel, vive_joints= vive_joints)
            Fused_skel= Fuse.Fusion()

            for ii in range(num_com):
                for kk in range(num_source):
                    np.savetxt(
                        './Temp_Result/' + "S" + format(subject, "03") + "C" + format(ii, "03") + "P"+format(kk,"03") + "R000A" + format(
                            action, "03") + "/frame_" + format(k,"06") + '.csv',
                        locals()[f"com{ii}_{kk}_joints"], delimiter=" ")

            p = np.asarray(Fused_skel.vJointPositions)
            # print(np.shape(p))
            #print(p)
            for i in range(27):
                if k > 1 and i != 0:
                    f.write(str(round(p[i, 0], 6)) + " " + str(round(p[i, 1], 6)) + " " + str(
                        round(p[i, 2], 6)) + " " + "0 0 0 0 0 0 0 0 0\n")

            k += 1
            if vis:

                points = ax.scatter(p[:, 0], p[:, 1], p[:, 2], c='b')
                plt.pause(0.001)
                plt.draw()
                points.remove()



            terminate_t = timeit.default_timer()
            delay_t = (1 / fps_cons) - (terminate_t - start_t)
            if delay_t > 0:
                time.sleep(delay_t)
            terminate_t = timeit.default_timer()
            FPS = 1 / (terminate_t - start_t)
            print("FPS: " + str(FPS))
            print("Frame: " + str(k))

            if keyboard.is_pressed("q"):
                for ii in range(num_com):
                    f.write(str(k - 2))
                for ii in range(num_com):
                    f.close
                plt.close()
                print("Stop")
                break



