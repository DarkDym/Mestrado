import random
import numpy
import rospy
from nav_msgs.msg import OccupancyGrid
import ast

resolution = 0.05
width = 4000
height = 4000
origin_x = -100.00
origin_y = -100.00

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

        with open('grid_valid_cells.txt','w') as valid_cells_grid:
            for x in range(len(valid_cells)):
                valid_cells_grid.write(str(valid_cells[x])+"\n")
        # isGoal_valid = False
        # while not isGoal_valid:
        goal = random.sample(valid_cells,1)    
        print(goal)
        # rospy.signal_shutdown    

    def open_grid_cells(self):
        self.valid_cells = []
        with open('grid_valid_cells.txt','r') as grid_file:
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

            Opções de teste:

        """
        # rospy.init_node('getLaneMap', anonymous=True)
        # grid_sub = rospy.Subscriber('/lane_map', OccupancyGrid, self.grid_callback)
        # rate = rospy.Rate(10)
        # rospy.spin()

        sowdc_regions = [[0.19,40.7,34.6,46.0],[-0.3,0.25,4.87,46.0],[-0.3,0.13,34.2,5.28],[29.6,0.13,34.6,45.6]]
        test_vector = [[1,2,3,4],[1,2,4,3],[1,3,2,4],[1,3,4,2],[1,4,2,3],[1,4,3,2],[2,1,3,4],[2,1,4,3],
                        [2,3,1,4],[2,3,4,1],[2,4,1,3],[2,4,3,1],[3,1,2,4],[3,1,4,2],[3,2,1,4],[3,2,4,1],[3,4,1,2],[3,4,2,1],
                        [4,1,2,3],[4,1,3,2],[4,2,1,3],[4,2,3,1],[4,3,1,2],[4,3,2,1]]
        test_vector_ind = [i for i in range(24)]
        random.seed(1)
        sample_test_vector = random.sample(test_vector_ind,24)
        print(sample_test_vector)
        self.open_grid_cells()
        blockage_threshold = 1.0

        blockage_file = open('ctl_draw_teste.txt','w')
        goal_file = open('objects_in_map.txt','w')

        for x in range(24):
            print("###############################################################################")
            for c in range(4):
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
                    while (aux_x2 - aux_x1) < blockage_threshold:
                        rand_goal_x1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                        rand_goal_x2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])
                        if rand_goal_x1 > rand_goal_x2:
                            aux_x1 = rand_goal_x2
                            aux_x2 = rand_goal_x1
                        else:
                            aux_x1 = rand_goal_x1
                            aux_x2 = rand_goal_x2
                    
                    # y1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]
                    # y2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]
                    while (aux_y2 - aux_y1) < blockage_threshold:
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
                    while (person[0] > 2*theta_r):
                        person = random.sample(range(1,100),1)
                    blockage_file.write(str(v0) + ";" + str(v1) + ";" + str(v2) + ";" + str(v3) + ";" + str(person[0]) + "\n")
                    isValidGoal_ = False
                    while not isValidGoal_:
                        goal = random.sample(self.valid_cells,1)
                        [gy,gx] = self.cell2odom(goal[0][0],goal[0][1])
                        
                        # print("################################################################")
                        # print("GX: " + str(gx) + " | GY: " + str(gy))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]) + " | aux_x1: " + str(aux_x1))
                        # print("aux_x2: " + str(aux_x2) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]) + " | y1: " + str(y1) + " | y2: " + str(y2))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]))
                        # print("################################################################")
                        if ((gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < aux_x1) or (gx > aux_x2 and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2])) and (gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]):
                            # print("ACHEI UM GOAL VALIDO PARA ESTA ITERACAO")
                            print("GX: " + str(gx) + " | GY: " + str(gy))
                            goal_file.write("160;"+str(gx)+";"+str(gy) + "\n")
                            print("QNT PERSON: " + str(person))
                            isValidGoal_ = True
                else:
                    while (aux_y2 - aux_y1) < blockage_threshold:
                        rand_goal_y1 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                        rand_goal_y2 = random.uniform(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1],sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])
                        if rand_goal_y1 > rand_goal_y2:
                            aux_y1 = rand_goal_y2
                            aux_y2 = rand_goal_y1
                        else:
                            aux_y1 = rand_goal_y1
                            aux_y2 = rand_goal_y2
                    # x1 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]
                    # x2 = sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]
                    while (aux_x2 - aux_x1) < blockage_threshold:
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
                    while (person[0] > 2*theta_r):
                        person = random.sample(range(1,100),1)
                    blockage_file.write(str(v0) + ";" + str(v1) + ";" + str(v2) + ";" + str(v3) + ";" + str(person[0]) + "\n")
                    isValidGoal_ = False
                    while not isValidGoal_:
                        goal = random.sample(self.valid_cells,1)
                        [gy,gx] = self.cell2odom(goal[0][0],goal[0][1])
                        
                        # print("################################################################")
                        # print("GX: " + str(gx) + " | GY: " + str(gy))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0]) + " | aux_x1: " + str(aux_x1))
                        # print("aux_x2: " + str(aux_x2) + " | sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1]) + " | x1: " + str(x1) + " | x2: " + str(x2))
                        # print("sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]: " + str(sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3]))
                        # print("################################################################")
                        if (gx > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][0] and gx < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][2]) and ((gy > sowdc_regions[test_vector[sample_test_vector[x]][c]-1][1] and gy < aux_y1) or (gy > aux_y2 and gy < sowdc_regions[test_vector[sample_test_vector[x]][c]-1][3])):
                            # print("ACHEI UM GOAL VALIDO PARA ESTA ITERACAO")
                            print("GX: " + str(gx) + " | GY: " + str(gy))
                            goal_file.write("160;"+str(gx)+";"+str(gy) + "\n")
                            print("QNT PERSON: " + str(person))
                            isValidGoal_ = True

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


calc = Calc_odom_cell()
calc.random_test()
# calc()