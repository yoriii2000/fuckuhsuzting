import serial
import time
import binascii
import re
import numpy as np
import matplotlib.pyplot as plt
import os


if __name__ == '__main__':

    # 通訊用資txt
    file_path = "connect_sensor.txt"
    file_exist_status = os.path.isdir(file_path)
    print("file_exist_status")
    print(file_exist_status)

    run = 0
    averageforce_file = open("force data/average1lineforce.txt", 'w').close()

    while 1:
        if os.path.exists(file_path):
            averageforce_file = open("force data/average1lineforce.txt", 'a+')
            # ti = np.genfromtxt(averageforce_file, dtype=None, comments='#', delimiter=' ')

            # 成功讀到force sensor回傳給C+開始移動
            return_robot_file = "return_robot.txt"

            # value = np.genfromtxt(file_path, dtype=None, comments='#', delimiter=' ')

            # path = 'text.txt'
            f = open(file_path, 'r')
            time.sleep(0.1)
            value = int(f.read())

            if value == 1:

                # for line_no in range(3):
                ser = serial.Serial()
                ser.port = "COM6"
                ser.baudrate = 115200
                ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
                ser.parity = serial.PARITY_NONE  # set parity check
                ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
                ser.timeout = 0.1  # non-block read 0.5s
                ser.writeTimeout = 0.1  # timeout for write 0.5s
                ser.xonxoff = False  # disable software flow control
                ser.rtscts = False  # disable hardware (RTS/CTS) flow control
                ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control

                times = np.array([])
                datafx = np.array([])
                datafy = np.array([])
                datafz = np.array([])
                dataF = np.array([])
                resultdata = np.array([])
                # starttime = time.time()

                plt.figure(figsize=(8, 3), dpi=95)
                plt.ion()
                plt.suptitle('Force Data', fontsize=20)
                plt.show()

                try:
                    ser.open()

                except Exception as ex:
                    print("open serial port error " + str(ex))
                    exit()

                cmd = b'c\r\n'
                # for line_no in range(3):
                if run % 3 == 0:
                    print('file number = ', int(run / 3), int(1))
                    text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(1)), 'w')
                    force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(1)), 'w')
                elif round(run / 3) == int(run / 3):
                    print('file number = ', int(run / 3), int(2))
                    text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(2)), 'w')
                    force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(2)), 'w')
                elif round(run / 3) == (int(run / 3) + 1):
                    print('file number = ', int(run / 3), int(3))
                    text_file = open("force data/datadetect{}_{}.txt".format(int(run / 3), int(3)), 'w')
                    force_file = open("force data/F_line{}_{}.txt".format(int(run / 3), int(3)), 'w')

                if ser.isOpen():
                    # send start command "c\r"

                    while (1):

                        ser.write(cmd)
                        print("write : %s" % cmd)

                        response = ser.read(90)
                        print("fread : %s" % response)
                        if len(response) > 50:
                            break

                    time.sleep(0.5)  # wait 0.5s
                    # read force command
                    starttime = time.time()

                    whilerun = 0
                    while (1):
                        try:
                            ser.flushInput()  # flush input buffer
                            ser.flushOutput()  # flush output buffer

                            # read 80 byte data
                            # response = ser.read(100)
                            response = ser.readline()

                            if len(response) > 50:
                                whilerun = whilerun + 1
                                # print('whilerun = ', whilerun)
                                print("read : %s" % response)

                                newres = response.decode('UTF-8', 'strict')
                                b = re.split(r'\s+', newres)

                                for i in range(b.count('')):
                                    b.remove('')

                                force = b[::2]
                                sensordata = force[:3]

                                fx = float(sensordata[0])
                                fy = float(sensordata[1])
                                fz = float(sensordata[2])

                                # 力感測總和力
                                F = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
                                F = (round(F*1000))/1000

                                forcedata = [fx, fy, fz]
                                print('forcedata = ', forcedata)

                                endtime = time.time()
                                processtime = endtime - starttime
                                print("耗時: {:.2f}秒".format(processtime))

                                X = round(processtime, 2)
                                Yx = fx
                                Yy = fy
                                Yz = fz
                                YF = F

                                strx = str(X)
                                strF = str(F)
                                newres = strx + ' s\t' + newres
                                Fxyz = strx + ' s\t' + strF + ' N\t\n'

                                text_file.write(newres)
                                force_file.write(Fxyz)

                                # print('text_file = ', type(newres))
                                if response == "\n":
                                    text_file.seek()
                                    force_file.seek()
                                    text_file.truncate()
                                    force_file.truncate()
                                text_file.flush()
                                force_file.flush()

                                resultdata = np.vstack([X, fx, fy, fz])
                                resultdata = resultdata.T

                                times = np.append(times, X)
                                datafx = np.append(datafx, Yx)
                                datafy = np.append(datafy, Yy)
                                datafz = np.append(datafz, Yz)
                                dataF = np.append(dataF, F)

                                plt.clf()

                                # plt.subplot(411)
                                # plt.xlabel("time(s)")
                                # # plt.legend(['Fx'])
                                # plt.ylabel("Fx(N)")
                                # plt.plot(times, datafx, color='tab:red')
                                # plt.grid()
                                #
                                # plt.subplot(412)
                                # plt.xlabel("time(s)")
                                # plt.ylabel("Fy(N)")
                                # plt.plot(times, datafy, color='tab:green')
                                # plt.grid()
                                #
                                # plt.subplot(413)
                                # plt.xlabel("time(s)")
                                # plt.ylabel("Fz(N)")
                                # plt.plot(times, datafz, color='tab:blue')
                                # plt.grid()

                                # plt.subplot(414)
                                plt.subplot()
                                plt.xlabel("time(s)")
                                plt.ylabel("F(N)")
                                plt.grid()
                                plt.plot(times, dataF, color='tab:red')

                                # 連到force sensor回傳1 1
                                if whilerun == 1:
                                    ru = open(return_robot_file, "w")
                                    ru.write(str(1))
                                    ru.close()
                                    print('return robot finish')

                                plt.pause(0.001)
                                plt.ioff()

                            # layer_finishornot = np.genfromtxt(file_path, dtype=None, comments='#', delimiter=' ')
                            backfile = open(file_path, 'r')
                            backvalue = int(backfile.read())
                            print()
                            if backvalue == 2:
                                oneline_endtime = time.time()

                                # ---------將原本的檔案覆蓋掉裡面的數值----------
                                f = open(file_path, "w")
                                f.write(str(0))
                                f.close()

                                # 離開force sensor回傳2 2
                                ru = open(return_robot_file, "w")
                                ru.write(str(0))
                                ru.close()

                                duringtime = oneline_endtime - starttime
                                print("oneline耗時: {:.2f}秒".format(duringtime))
                                duringtime = round(duringtime, 2)
                                strduringtime = str(duringtime)
                                oneline_time = strduringtime + ' s\t\n'
                                averageforce_file.write(oneline_time)

                                break
                                # ------------------------------------
                                # sys.exit()
                            run = run + 1

                        except Exception as e1:
                            print("communicating error " + str(e1))

                    ser.close()

                else:
                    print("open serial port error")
            else:
                print("wait")
        else:
            print("檔案路徑不存在。")






