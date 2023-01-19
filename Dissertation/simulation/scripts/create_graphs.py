import numpy
import matplotlib.pyplot as plt
import os
import sys

class ReadLogs:
    def __init__(self) -> None:
        # pass
        basepath = './Dissertation/simulation/Logs_18-01/Time_Logs/'
        ST_NB_0 = []
        ST_NB_1 = []
        ST_WB_0 = []
        ST_WB_1 = []
        RT_NB_0 = []
        RT_NB_1 = []
        RT_WB_0 = []
        RT_WB_1 = []
        for logfile in os.listdir(basepath):
            if os.path.isfile(os.path.join(basepath,logfile)):
                print(logfile)
                with open(basepath + logfile, 'r') as logs:
                    all_lines = logs.readlines()
                    cont = 0
                    RT = []
                    ST = []
                    for line in all_lines:
                        # print(cont)
                        if cont <= 1 or cont >= 6:
                            cont += 1
                            if cont == 10:
                                cont = 0
                        else:
                            if line.find('REAL TIME') > 0:
                                splited = line.split('\n')
                                # print(splited)
                                # print(line)
                                RT.append(splited[0])
                                t = splited[0].split('= ')
                                # print(splited[0].split('= '))
                                secs = t[1].split('[')
                                # print(secs[0])
                            elif line.find('SIMULATION TIME') > 0:
                                splited = line.split('\n')
                                # print(splited)
                                # print(line)
                                ST.append(splited[0])
                                t = splited[0].split('= ')
                                # print(splited[0].split('= '))
                                secs = t[1].split('[')
                                # print(secs[0])
                            cont += 1
            # print(RT)
            # print(ST)
            RT_sec = []
            ST_sec = []
            for x in range(0,len(RT)):
                splited = RT[x].split('\n')
                t = splited[0].split('= ')
                secs = t[1].split('[')
                RT_sec.append(int(secs[0]))
            for x in range(0,len(ST)):
                splited = ST[x].split('\n')
                t = splited[0].split('= ')
                secs = t[1].split('[')
                ST_sec.append(int(secs[0]))
            
            cont = 0
            cont_aux = 0
            RT_sec_wb = []
            RT_sec_nb = []
            ST_sec_wb = []
            ST_sec_nb = []
            # print(len(RT_sec))
            # print(RT_sec)
            # print(len(ST_sec))
            # print(ST_sec)
            for x in range(0,len(RT_sec)):
                if (cont % 2) == 0 :
                    if cont_aux < 1:
                        RT_sec_nb.append(RT_sec[x])
                        cont_aux += 1
                    else:
                        RT_sec_nb.append(RT_sec[x])
                        cont_aux = 0
                        cont += 1
                else:
                    if cont_aux < 1:
                        RT_sec_wb.append(RT_sec[x])
                        cont_aux += 1
                    else:
                        RT_sec_wb.append(RT_sec[x])
                        cont_aux = 0
                        cont += 1
            # print(len(RT_sec_wb))
            # print(RT_sec_wb)
            # print(len(RT_sec_nb))
            # print(RT_sec_nb)
            cont = 0
            cont_aux = 0
            for x in range(0,len(ST_sec)):
                if (cont % 2) == 0 :
                    if cont_aux < 1:
                        ST_sec_nb.append(ST_sec[x])
                        cont_aux += 1
                    else:
                        ST_sec_nb.append(ST_sec[x])
                        cont_aux = 0
                        cont += 1
                else:
                    if cont_aux < 1:
                        ST_sec_wb.append(ST_sec[x])
                        cont_aux += 1
                    else:
                        ST_sec_wb.append(ST_sec[x])
                        cont_aux = 0
                        cont += 1
            line_RT_wb_0 = []
            line_RT_wb_1 = []
            line_RT_nb_0 = []
            line_RT_nb_1 = []
            line_ST_wb_0 = []
            line_ST_wb_1 = []
            line_ST_nb_0 = []
            line_ST_nb_1 = []
            for x in range(0,len(RT_sec_wb)):
                if (x % 2) == 0 :
                    line_RT_wb_0.append(RT_sec_wb[x])
                    line_RT_nb_0.append(RT_sec_nb[x])
                else:
                    line_RT_wb_1.append(RT_sec_wb[x])
                    line_RT_nb_1.append(RT_sec_nb[x])
            x_axis = [0,1,2,3,4,5,6,7,8,9]
            # print(len(line_RT_wb_0))
            # plt.plot(x_axis,line_RT_wb_0,'ro',x_axis,line_RT_nb_0,'bo')
            # plt.axis([0, 10, 0, 200])
            # plt.xticks(range(0,10))
            # plt.yticks(range(0,230,10))
            # plt.show()
            # print(len(ST_sec_wb))
            # print(ST_sec_wb)
            # print(len(ST_sec_nb))
            # print(ST_sec_nb)
            for x in range(0,len(ST_sec_wb)):
                if (x % 2) == 0 :
                    line_ST_wb_0.append(ST_sec_wb[x])
                    line_ST_nb_0.append(ST_sec_nb[x])
                else:
                    line_ST_wb_1.append(ST_sec_wb[x])
                    line_ST_nb_1.append(ST_sec_nb[x])
            ST_NB_0.append(line_ST_nb_0)
            ST_NB_1.append(line_ST_nb_1)
            ST_WB_0.append(line_ST_wb_0)
            ST_WB_1.append(line_ST_wb_1)
            RT_NB_0.append(line_RT_nb_0)
            RT_NB_1.append(line_RT_nb_1)
            RT_WB_0.append(line_RT_wb_0)
            RT_WB_1.append(line_RT_wb_1)
        x_axis = [0,1,2,3,4,5,6,7,8,9]
        # plt.plot(x_axis,ST_WB_0[0],'r--',x_axis,ST_WB_0[1],'b--',x_axis,ST_WB_0[2],'g--')
        # plt.xticks(range(0,10))
        # plt.yticks(range(0,230,10))
        # plt.title("SIMULATION TIME | WITH BLOCKAGE | A->B")
        # plt.xlabel("INDICE")
        # plt.ylabel("TEMPO [s]")
        # plt.show()
        
        # plt.plot(x_axis,ST_WB_1[0],'r--',x_axis,ST_WB_1[1],'b--',x_axis,ST_WB_1[2],'g--')
        # plt.xticks(range(0,10))
        # plt.yticks(range(0,230,10))
        # plt.title("SIMULATION TIME | WITH BLOCKAGE | B->A")
        # plt.xlabel("INDICE")
        # plt.ylabel("TEMPO [s]")
        # plt.show()

        plt.plot(x_axis,ST_WB_0[2],'ro-',x_axis,RT_WB_0[2],'co-',x_axis,ST_WB_0[0],'gs--',x_axis,RT_WB_0[0],'ys--',x_axis,ST_WB_0[5],'bv:',x_axis,RT_WB_0[5],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,130,10))
        plt.title(str(sys.argv[1]) + " | [ SIMULATION X REAL ] TIME | WITH BLOCKAGE | A->B")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 1p','REAL TIME 1p','SIMULATION TIME 2p','REAL TIME 2p','SIMULATION TIME 3p','REAL TIME 3p'])
        plt.savefig('sXr_wb_ab_pt1.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_WB_1[2],'ro-',x_axis,RT_WB_1[2],'co-',x_axis,ST_WB_1[0],'gs--',x_axis,RT_WB_1[0],'ys--',x_axis,ST_WB_1[5],'bv:',x_axis,RT_WB_1[5],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,130,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | WITH BLOCKAGE | B->A")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 1p','REAL TIME 1p','SIMULATION TIME 2p','REAL TIME 2p','SIMULATION TIME 3p','REAL TIME 3p'])
        plt.savefig('sXr_wb_ba_pt1.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_NB_0[2],'ro-',x_axis,RT_NB_0[2],'co-',x_axis,ST_NB_0[0],'gs--',x_axis,RT_NB_0[0],'ys--',x_axis,ST_NB_0[5],'bv:',x_axis,RT_NB_0[5],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,230,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | NO BLOCKAGE | A->B")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 1p','REAL TIME 1p','SIMULATION TIME 2p','REAL TIME 2p','SIMULATION TIME 3p','REAL TIME 3p'])
        plt.savefig('sXr_nb_ab_pt1.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_NB_1[2],'ro-',x_axis,RT_NB_1[2],'co-',x_axis,ST_NB_1[0],'gs--',x_axis,RT_NB_1[0],'ys--',x_axis,ST_NB_1[5],'bv:',x_axis,RT_NB_1[5],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,230,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | NO BLOCKAGE | B->A")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 1p','REAL TIME 1p','SIMULATION TIME 2p','REAL TIME 2p','SIMULATION TIME 3p','REAL TIME 3p'])
        plt.savefig('sXr_nb_ba_pt1.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_WB_0[3],'ro-',x_axis,RT_WB_0[3],'co-',x_axis,ST_WB_0[4],'gs--',x_axis,RT_WB_0[4],'ys--',x_axis,ST_WB_0[1],'bv:',x_axis,RT_WB_0[1],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,130,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | WITH BLOCKAGE | A->B")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 4p','REAL TIME 4p','SIMULATION TIME 5p','REAL TIME 5p','SIMULATION TIME 4p_WO','REAL TIME 4p_WO'])
        plt.savefig('sXr_wb_ab_pt2.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_WB_1[3],'ro-',x_axis,RT_WB_1[3],'co-',x_axis,ST_WB_1[4],'gs--',x_axis,RT_WB_1[4],'ys--',x_axis,ST_WB_1[1],'bv:',x_axis,RT_WB_1[1],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,130,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | WITH BLOCKAGE | B->A")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 4p','REAL TIME 4p','SIMULATION TIME 5p','REAL TIME 5p','SIMULATION TIME 4p_WO','REAL TIME 4p_WO'])
        plt.savefig('sXr_wb_ba_pt2.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_NB_0[3],'ro-',x_axis,RT_NB_0[3],'co-',x_axis,ST_NB_0[4],'gs--',x_axis,RT_NB_0[4],'ys--',x_axis,ST_NB_0[1],'bv:',x_axis,RT_NB_0[1],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,230,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | NO BLOCKAGE | A->B")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 4p','REAL TIME 4p','SIMULATION TIME 5p','REAL TIME 5p','SIMULATION TIME 4p_WO','REAL TIME 4p_WO'])
        plt.savefig('sXr_nb_ab_pt2.png',dpi=720)
        plt.show()

        plt.plot(x_axis,ST_NB_1[3],'ro-',x_axis,RT_NB_1[3],'co-',x_axis,ST_NB_1[4],'gs--',x_axis,RT_NB_1[4],'ys--',x_axis,ST_NB_1[1],'bv:',x_axis,RT_NB_1[1],'mv:')
        plt.xticks(range(0,10))
        plt.yticks(range(50,230,10))
        plt.title(str(sys.argv[1]) + " |  SIMULATION X REAL ] TIME | NO BLOCKAGE | B->A")
        plt.xlabel("INDICE")
        plt.ylabel("TEMPO [s]")
        plt.legend(['SIMULATION TIME 4p','REAL TIME 4p','SIMULATION TIME 5p','REAL TIME 5p','SIMULATION TIME 4p_WO','REAL TIME 4p_WO'])
        plt.savefig('sXr_nb_ba_pt2.png',dpi=720)
        plt.show()
        
rl = ReadLogs()