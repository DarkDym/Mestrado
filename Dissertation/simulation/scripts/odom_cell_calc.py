import random
import numpy
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import ast
from itertools import permutations

resolution = 0.05
width = 4000
height = 4000
origin_x = -100.00
origin_y = -100.00
p_count = 0
block_points = []
blockage = []

class Calc_odom_cell:
    def __init__(self):
        print("START")
        # close_op = False

        # while(not close_op):
        #     print("1 - ODOM_2_CELL")
        #     print("2 - CELL_2_ODOM")
        #     mode = input(":")
        #     if int(mode) == 1:
        #         ix = input("ODOM_X: ")
        #         iy = input("ODOM_Y: ")
        #         cell_x = float(ix)/resolution - origin_x/resolution
        #         cell_y = float(iy)/resolution - origin_x/resolution
        #         print("RESULTADO [CELL_X ; CELL_Y] : [ " + str(cell_x) + " ; " + str(cell_y) + " ]")
        #     elif int(mode) == 2:
        #         ix = input("CELL_X: ")
        #         iy = input("CELL_Y: ")
        #         odom_x = (int(ix) + origin_x/resolution) * resolution
        #         odom_y = (int(iy) + origin_y/resolution) * resolution
        #         print("RESULTADO [ODOM_X ; ODOM_Y] : [ " + str(odom_x) + " ; " + str(odom_y) + " ]")
        #     elif int(mode) == 3:
        #         # sample_list = [1, 2, 3, 4]
        #         # list_combinations = list()

        #         # for n in range(len(sample_list)):
        #         n = input("QNT DE AREAS NO MAPA: ")
        #         list_combinations = list(permutations(range(1,int(n)+1)))

        #         print(list_combinations)
        #     else:
        #         print("FECHANDO")
        #         close_op = True
    
    
    def odom2cell(self,ix,iy):
        cell_x = float(ix)/resolution - origin_x/resolution
        cell_y = float(iy)/resolution - origin_x/resolution
        return (int(cell_x),int(cell_y))
    def cell2odom(self,ix,iy):
        odom_x = (int(ix) + origin_x/resolution) * resolution
        odom_y = (int(iy) + origin_y/resolution) * resolution
        return (float(odom_x),float(odom_y))

    def grid_callback(self,grid_msg):
        # print(grid_msg.data[0])
        grid = numpy.zeros((width,height))
        print(len(grid_msg.data))
        valid_cells = []
        for x in range(width):
            multi = x * width
            for y in range(height):
                index = y + multi
                grid[x][y] = grid_msg.data[index]
                if grid_msg.data[index] == 0:
                    valid_cells.append((x,y))

        with open('new_grid_valid_cells.txt','w') as valid_cells_grid:
            for x in range(len(valid_cells)):
                valid_cells_grid.write(str(valid_cells[x])+"\n")
        # isGoal_valid = False
        # while not isGoal_valid:
        goal = random.sample(valid_cells,1)    
        print(goal)
        print("PROCESSO DE CRIACAO DO ARQUIVO FINALIZADA!")
        rospy.signal_shutdown("TERMINEI o ARQUIVO")
        exit()
        # rospy.signal_shutdown    
    
    def point_callback(self,point_msg):
        global p_count
        global block_points
        global blockage
        print("PONTO X: " + str(point_msg.point.x))
        print("PONTO Y: " + str(point_msg.point.y))
        block_points.append((point_msg.point.x,point_msg.point.y))
        p_count += 1
        if p_count < 2:
            print("SELECIONE O PROXIMO PONTOS")
        else:
            print("CRIANDO O BLOQUEIO NAS SEGUINTES COORDENADAS")
            print("X1: " + str(block_points[0][0]) + " Y1: " + str(block_points[0][1]))
            (cell_x1,cell_y1) = self.odom2cell(block_points[0][0],block_points[0][1])
            print("CELL_X1: " + str(cell_x1) + " CELL_Y1: " + str(cell_y1))
            print("X2: " + str(block_points[1][0]) + " Y2: " + str(block_points[1][1]))
            (cell_x2,cell_y2) = self.odom2cell(block_points[1][0],block_points[1][1])
            print("CELL_X2: " + str(cell_x2) + " CELL_Y2: " + str(cell_y2))
            blockage.append((cell_x1,cell_y1,cell_x2,cell_y2))
            option = input("QUER CRIAR MAIS BLOQUEIOS? 1 - SIM | 2 - NÃO --> ")
            if int(option) == 1:
                p_count = 0
                block_points.clear()
                print("PODE ESCOLHER OS PROXIMOS PONTOS")
            elif int(option) == 2:
                print(blockage)
                rospy.signal_shutdown("TERMINEI")
                exit()


    def open_grid_cells(self,regions):
        self.valid_cells = []
        with open('grid_valid_cells_'+str(regions)+'.txt','r') as grid_file:
            lines = grid_file.readlines()
            for line in lines:
                splited = line.split('\n')
                # self.valid_cells.append(splited[0])
                result = ast.literal_eval(splited[0])
                self.valid_cells.append(result)
                # print(result)

    
    def random_test(self):
        """
            Regiões do mapa SOWDC:
                [1] - [(0.19;41.1);(0.19;46.0);(34.6;45.6);(34.6;40.7)] --> [0.19;40.7][34.6;46.0]
                [2] - [(0.19;46.0);(4.87;46.0);(4.5;0.25);(-0.3;0.25)] --> [-0.3;0.25][4.87;46.0]
                [3] - [(-0.3;0.25);(-0.3;5.28);(34.2;5.26);(34.2;0.13)] --> [-0.3;0.13][34.2;5.28]
                [4] - [(29.6;0.14);(34.2;0.13);(34.6;45.6);(30.0;45.6)] --> [29.6;0.13][34.6;45.6]

            Regiões do mapa SOWDC_2P:
                [1] - [(0.19;41.1);(0.19;46.0);(34.6;45.6);(34.6;40.7)] --> [0.19;40.7][34.6;46.0]
                [2] - [(0.19;46.0);(4.87;46.0);(4.5;0.25);(-0.3;0.25)] --> [-0.3;0.25][4.87;46.0]
                [3] - [(-0.3;0.25);(-0.3;5.28);(34.2;5.26);(34.2;0.13)] --> [-0.3;0.13][34.2;5.28]
                [4] - [(29.6;0.14);(34.2;0.13);(34.6;45.6);(30.0;45.6)] --> [29.6;0.13][34.6;45.6]
                [5] - [(-0.146;23.6);(-0,0794;28.2);(34.4;27.9);(34.3;23.3)] --> [-0.146;23.6][34.4;28.2]

            Regiões do mapa SOWDC_4P:
                [1] - [(0.19;41.1);(0.19;46.0);(34.6;45.6);(34.6;40.7)] --> [0.19;40.7][34.6;46.0]
                [2] - [(0.19;46.0);(4.87;46.0);(4.5;0.25);(-0.3;0.25)] --> [-0.3;0.25][4.87;46.0]
                [3] - [(-0.3;0.25);(-0.3;5.28);(34.2;5.26);(34.2;0.13)] --> [-0.3;0.13][34.2;5.28]
                [4] - [(29.6;0.14);(34.2;0.13);(34.6;45.6);(30.0;45.6)] --> [29.6;0.13][34.6;45.6]
                [5] - [(-0.146;23.6);(-0,0794;28.2);(34.4;27.9);(34.3;23.3)] --> [-0.146;23.6][34.4;28.2]
                [6] - [(15.3;45.8);(21.3;45.7);(20.5;0.251);(14.5;0.168)] --> [14.5;0.168][21.3;45.7]

        """
        map_mode = input("1: GERAR ARQUIVO DE CÉLULAS LIVRES | 2: GERAR ARQUIVOS DE SIMULAÇÃO | 3: GERAR BLOQUEIOS A PARTIR DO CLICKE_POINT | 4: CALCULAR ODOM2CELL CELL2ODOM-->")
        if int(map_mode) == 1 :
            rospy.init_node('getLaneMap', anonymous=True)
            grid_sub = rospy.Subscriber('/lane_map', OccupancyGrid, self.grid_callback)
            rate = rospy.Rate(10)
            rospy.spin()
        elif int(map_mode) == 2 :
            choice = input("1: SOWDC  |  2: SOWDC_2P  |  3: SOWDC_4P | :")
            if int(choice) == 1:
                random.seed(1)
                sowdc_regions = [[0.19,40.7,34.6,46.0],[-0.3,0.25,4.97,46.0],[-0.3,0.13,34.2,5.28],[29.6,0.13,34.6,45.6]]
                test_vector = [[1,2,3,4],[1,2,4,3],[1,3,2,4],[1,3,4,2],[1,4,2,3],[1,4,3,2],[2,1,3,4],[2,1,4,3],
                            [2,3,1,4],[2,3,4,1],[2,4,1,3],[2,4,3,1],[3,1,2,4],[3,1,4,2],[3,2,1,4],[3,2,4,1],[3,4,1,2],[3,4,2,1],
                            [4,1,2,3],[4,1,3,2],[4,2,1,3],[4,2,3,1],[4,3,1,2],[4,3,2,1]]
                test_vector = list(permutations(range(1,5)))
                test_vector_ind = [i for i in range(24)]
                sample_test_vector = random.sample(test_vector_ind,len(test_vector))
                tam_test = len(test_vector)
                qnt_regions = 4
            elif int(choice) == 2:
                random.seed(1)
                sowdc_regions = [[0.19,40.7,34.6,46.0],[-0.3,0.25,4.97,46.0],[-0.3,0.13,34.2,5.28],[29.6,0.13,34.6,45.6],[-0.146,23.6,34.4,28.2]]
                test_vector_2p = list(permutations(range(1,6)))
                test_vector_ind_2p = [i for i in range(len(test_vector_2p))]
                # sample_test_vector_2p = random.sample(test_vector_ind_2p,len(test_vector_2p))
                sample_test_vector_2p = random.sample(test_vector_ind_2p,24)
                sample_test_vector = sample_test_vector_2p
                # tam_test = len(test_vector_2p)
                tam_test = 24
                qnt_regions = 5
                test_vector = test_vector_2p
            elif int(choice) == 3:
                random.seed(1)
                sowdc_regions = [[0.19,40.7,34.6,46.0],[-0.3,0.25,4.97,46.0],[-0.3,0.13,34.2,5.28],[29.6,0.13,34.6,45.6],[-0.146,23.6,34.4,28.2],[14.5,0.168,21.3,45.7]]
                test_vector_4p = list(permutations(range(1,7)))
                test_vector_ind_4p = [i for i in range(len(test_vector_4p))]
                # sample_test_vector_4p = random.sample(test_vector_ind_4p,len(test_vector_4p))
                sample_test_vector_4p = random.sample(test_vector_ind_4p,24)
                sample_test_vector = sample_test_vector_4p
                # tam_test = len(test_vector_4p)
                tam_test = 24
                qnt_regions = 6
                test_vector = test_vector_4p
            else:
                print("FINALIZANDO")
            
            # print(sample_test_vector)
            if int(choice) == 1:
                self.open_grid_cells(4)
                blockage_file = open('ctldraw_teste_4R.txt','w')
                goal_file = open('objects_in_map_4R.txt','w')
                robot_pose_file = open('robot_pose_4R.txt', 'w')
            elif int(choice) == 2:
                self.open_grid_cells(5)
                blockage_file = open('ctldraw_teste_5R_MOD.txt','w')
                goal_file = open('objects_in_map_5R_MOD.txt','w')
                robot_pose_file = open('robot_pose_5R_MOD.txt', 'w')
            elif int(choice) == 3:
                self.open_grid_cells(6)
                blockage_file = open('ctldraw_teste_6R_MOD.txt','w')
                goal_file = open('objects_in_map_6R_MOD.txt','w')
                robot_pose_file = open('robot_pose_6R_MOD.txt', 'w')
            else:
                exit()
            blockage_threshold = 1.0

            for _ in range(2):
                for x in range(tam_test):
                    print("###############################################################################")
                    for c in range(qnt_regions):
                        # print(sowdc_regions[c])
                        # print("SOWDC: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1]) + " | " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]) + " | " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1])
                            #   + " | " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]) + " | " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]))

                        # print("[" + str(x) + " ; " + str(c) + "]")
                        # print("SAMPLE X: " + str(sample_test_vector[x]))
                        # print("TEST_VECOTR: " + str(test_vector[sample_test_vector[x]][c]))
                        # print(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0])
                        # print(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1])
                        # print(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                        # print(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                        # rand_goal_x = random.sample(range(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]), 1)
                        # rand_goal_y = random.sample(range(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]), 1)
                        aux_x1 = 0
                        aux_x2 = 0
                        aux_y1 = 0
                        aux_y2 = 0
                        if test_vector[sample_test_vector[x]][c] % 2 == 0:
                            # if (test_vector[sample_test_vector[x]][c] == 2):
                            aux_x1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]
                            aux_x2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]
                            # else:

                            # while (aux_x2 - aux_x1) < blockage_threshold:
                            #     rand_goal_x1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                            #     rand_goal_x2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                            #     if rand_goal_x1 > rand_goal_x2:
                            #         aux_x1 = rand_goal_x2
                            #         aux_x2 = rand_goal_x1
                            #     else:
                            #         aux_x1 = rand_goal_x1
                            #         aux_x2 = rand_goal_x2
                            
                            # y1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]
                            # y2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]
                            rand_goal_y1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                            rand_goal_y2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])                                        
                            if rand_goal_y1 > rand_goal_y2:
                                aux_y1 = rand_goal_y2
                                aux_y2 = rand_goal_y1
                            else:
                                aux_y1 = rand_goal_y1
                                aux_y2 = rand_goal_y2
                            while not((aux_y2 - aux_y1) >= blockage_threshold and (aux_y2 - aux_y1) <= 10*blockage_threshold):
                                rand_goal_y1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                                rand_goal_y2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])                                        
                                if rand_goal_y1 > rand_goal_y2:
                                    aux_y1 = rand_goal_y2
                                    aux_y2 = rand_goal_y1
                                else:
                                    aux_y1 = rand_goal_y1
                                    aux_y2 = rand_goal_y2
                            print("[" + str(x) + " ; " + str(test_vector[sample_test_vector[x]][c]) + "] | BLOCK : [ (" + str(aux_x1) + " ; " + str(aux_y1) + ") ; (" + str(aux_x2) + " ; " + str(aux_y2) + ") ]")
                            print("BLOCK CELLS: [ " + str(self.odom2cell(aux_x1,aux_y1)) + " " + str(self.odom2cell(aux_x2,aux_y2)) + " ]")
                            [v0,v1] = self.odom2cell(aux_x1,aux_y1)
                            [v2,v3] = self.odom2cell(aux_x2,aux_y2)
                            person = random.sample(range(1,100),1)
                            theta_r = (aux_x2-aux_x1) * (aux_y2-aux_y1)
                            print("THETA_R: " + str(theta_r) + " | x: " + str(2*theta_r) + " | PERSON: " + str(person[0]))
                            # while (person[0] > 2*theta_r):
                            #     person = random.sample(range(1,100),1)
                            blockage_file.write(str(v0) + ";" + str(v1) + ";" + str(v2) + ";" + str(v3) + ";" + str(person[0]) + "\n")
                            isValidGoal_ = False
                            goal_x = 0
                            goal_y = 0
                            while not isValidGoal_:
                                # print("ESTOU AQUI PAR")
                                goal = random.sample(self.valid_cells,1)
                                [gy,gx] = self.cell2odom(goal[0][0],goal[0][1])
                                
                                # print("################################################################")
                                # print("GX: " + str(gx) + " | GY: " + str(gy))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]) + " | aux_x1: " + str(aux_x1))
                                # print("aux_x2: " + str(aux_x2) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]) + " | y1: " + str(aux_y1) + " | y2: " + str(aux_y2))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]))
                                # print("################################################################")
                                # if ((gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < aux_x1) or (gx > aux_x2 and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])) and (gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]):
                                if (((gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < aux_y1) or (gy > aux_y2 and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])) and (gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])):
                                    # print("ACHEI UM GOAL VALIDO PARA ESTA ITERACAO")
                                    print("GX: " + str(gx) + " | GY: " + str(gy))
                                    goal_file.write("160;"+str(gx)+";"+str(gy) + "\n")
                                    print("QNT PERSON: " + str(person))
                                    isValidGoal_ = True
                                    goal_x = gx
                                    goal_y = gy
                            rx = 0
                            ry = 0
                            if (goal_y > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and goal_y < aux_y1):
                                if (((aux_y1 - goal_y) + aux_y2 > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])):
                                    if (1 + aux_y2 < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]):
                                        dy = 1
                                    else:
                                        dy = (aux_y2 + sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])/2
                                else:    
                                    dy = aux_y1 - goal_y
                                ry = aux_y2 + dy
                                rx = goal_x
                            elif (goal_y > aux_y2 and goal_y < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]):
                                if ((aux_y1 - (goal_y - aux_y2) < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1])):
                                    if (aux_y1 - 1 > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]):
                                        dy = 1
                                    else:
                                        dy = (aux_y1 + sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1])/2
                                else:
                                    dy = goal_y - aux_y2
                                ry = aux_y1 - dy
                                rx = goal_x
                            robot_pose_file.write(str(rx)+";"+str(ry)+"\n")
                        else:
                            aux_y1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]
                            aux_y2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]
                            # while (aux_y2 - aux_y1) < blockage_threshold:
                            #     rand_goal_y1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                            #     rand_goal_y2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                            #     if rand_goal_y1 > rand_goal_y2:
                            #         aux_y1 = rand_goal_y2
                            #         aux_y2 = rand_goal_y1
                            #     else:
                            #         aux_y1 = rand_goal_y1
                            #         aux_y2 = rand_goal_y2
                            # x1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]
                            # x2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]
                            rand_goal_x1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                            rand_goal_x2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                            if rand_goal_x1 > rand_goal_x2:
                                aux_x1 = rand_goal_x2
                                aux_x2 = rand_goal_x1
                            else:
                                aux_x1 = rand_goal_x1
                                aux_x2 = rand_goal_x2
                            while not((aux_x2 - aux_x1) >= blockage_threshold and (aux_x2 - aux_x1) <= 10*blockage_threshold):
                                rand_goal_x1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                                rand_goal_x2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                                if rand_goal_x1 > rand_goal_x2:
                                    aux_x1 = rand_goal_x2
                                    aux_x2 = rand_goal_x1
                                else:
                                    aux_x1 = rand_goal_x1
                                    aux_x2 = rand_goal_x2
                            
                            
                            print("[" + str(x) + " ; " + str(test_vector[sample_test_vector[x]][c]) + "] | BLOCK : [ (" + str(aux_x1) + " ; " + str(aux_y1) + ") ; (" + str(aux_x2) + " ; " + str(aux_y2) + ") ]")
                            print("BLOCK CELLS: [ " + str(self.odom2cell(aux_x1,aux_y1)) + " " + str(self.odom2cell(aux_x2,aux_y2)) + " ]")
                            [v0,v1] = self.odom2cell(aux_x1,aux_y1)
                            [v2,v3] = self.odom2cell(aux_x2,aux_y2)
                            person = random.sample(range(1,100),1)
                            theta_r = (aux_x2-aux_x1) * (aux_y2-aux_y1)
                            print("THETA_R: " + str(theta_r) + " | x: " + str(2*theta_r) + " | PERSON: " + str(person[0]))
                            # while (person[0] > 2*theta_r):
                            #     person = random.sample(range(1,100),1)
                            blockage_file.write(str(v0) + ";" + str(v1) + ";" + str(v2) + ";" + str(v3) + ";" + str(person[0]) + "\n")
                            isValidGoal_ = False
                            goal_x = 0
                            goal_y = 0
                            while not isValidGoal_:
                                # print("ESTOU AQUI IMPAR")
                                goal = random.sample(self.valid_cells,1)
                                [gy,gx] = self.cell2odom(goal[0][0],goal[0][1])
                                
                                # print("################################################################")
                                # print("GX: " + str(gx) + " | GY: " + str(gy))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]) + " | aux_x1: " + str(aux_x1))
                                # print("aux_x2: " + str(aux_x2) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]) + " | x1: " + str(x1) + " | x2: " + str(x2))
                                # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]))
                                # print("################################################################")
                                # if (gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]) and ((gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < aux_y1) or (gy > aux_y2 and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])):
                                # print("GX: " + str(gx) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]:" + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]))
                                # print("aux_x1: " + str(aux_x1) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]:" + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]))
                                # print("aux_x2: " + str(aux_x2))
                                if (((gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < aux_x1) or (gx > aux_x2 and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])) and (gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])):
                                    # print("ACHEI UM GOAL VALIDO PARA ESTA ITERACAO")
                                    print("GX: " + str(gx) + " | GY: " + str(gy))
                                    goal_file.write("160;"+str(gx)+";"+str(gy) + "\n")
                                    print("QNT PERSON: " + str(person))
                                    isValidGoal_ = True
                                    goal_x = gx
                                    goal_y = gy
                            rx = 0
                            ry = 0
                            if (goal_x > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and goal_x < aux_x1):
                                if (((aux_x1 - goal_x) + aux_x2 > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])):
                                    if ((aux_x2 + 1) < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]):
                                        dx = 1
                                    else:
                                        dx = (aux_x2 + sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])/2
                                else:
                                    dx = aux_x1 - goal_x
                                rx = aux_x2 + dx
                                ry = goal_y
                            elif (goal_x > aux_x2 and goal_x < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]):
                                if ((aux_x1 - (goal_x - aux_x2) < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0])):
                                    if ((aux_x1 - 1) > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]):
                                        dx = 1
                                    else:
                                        dx = (aux_x1 + sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0])/2
                                else:
                                    dx = goal_x - aux_x2
                                rx = aux_x1 - dx
                                ry = goal_y
                            robot_pose_file.write(str(rx)+";"+str(ry)+"\n")

                        # rand_goal_x = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                        # rand_goal_y = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                        # # print("----------------------------------------------------------------")
                        # print("[" + str(x) + " ; " + str(c) + "] | GOAL : [ " + str(rand_goal_x) + " ; " + str(rand_goal_y) + " ]")
                        print("----------------------------------------------------------------")
                    print("###############################################################################")
                
                
                # print(self.valid_cells[1])
                
                    # print()
                # goal = random.sample(self.valid_cells,1)
                # print("GOAL: " + str(goal) + " | X : " + str(goal[0][0]) + " | Y : " + str(goal[0][1]))
                # [gx,gy] = self.cell2odom(goal[0][0],goal[0][1])
                # print("ODOM GOAL : " + str(gx) + " | " + str(gy))
                # pass
                # rospy.spin()
        elif int(map_mode) == 3:
            rospy.init_node('getClickedPoint', anonymous=True)
            grid_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
            rate = rospy.Rate(10)
            print("SELECIONE PELO RVIZ OS PONTOS NO QUAL DESEJA CRIAR UM BLOQUEIO.")
            rospy.spin()
        elif int(map_mode) == 4:
            close_op = False

            while(not close_op):
                print("1 - ODOM_2_CELL")
                print("2 - CELL_2_ODOM")
                mode = input(":")
                if int(mode) == 1:
                    ix = input("ODOM_X: ")
                    iy = input("ODOM_Y: ")
                    cell_x = float(ix)/resolution - origin_x/resolution
                    cell_y = float(iy)/resolution - origin_x/resolution
                    print("RESULTADO [CELL_X ; CELL_Y] : [ " + str(cell_x) + " ; " + str(cell_y) + " ]")
                elif int(mode) == 2:
                    ix = input("CELL_X: ")
                    iy = input("CELL_Y: ")
                    odom_x = (int(ix) + origin_x/resolution) * resolution
                    odom_y = (int(iy) + origin_y/resolution) * resolution
                    print("RESULTADO [ODOM_X ; ODOM_Y] : [ " + str(odom_x) + " ; " + str(odom_y) + " ]")
                else:
                    print("FECHANDO")
                    close_op = True
        else:
            exit()


calc = Calc_odom_cell()
calc.random_test()
# calc()