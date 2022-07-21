#!/usr/bin/env python
from numpy import empty
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse



class gmap:

    def __init__(self, get_map_topic_name, map_service):
        self.map = OccupancyGrid()
        self.wall_gap_x  = 0
        self.wal_gap_y   = 0
        self.get_map_topic_name = get_map_topic_name
        self.get_map_service    = map_service

    def map_is_enclosed(self, print_data, print_process, print_final_map):
        part_one_map = self.map_is_enclosed_initial(print_data)

        # Optional printing to showcase the process
        if print_process:
            self.print_map_terminal(part_one_map)

        return self.map_is_enclosed_advanced(part_one_map, print_process, print_final_map)

    def map_is_enclosed_initial(self, print_data): 
        # Make a shallow copy of our map 
        map = []
        for index in self.map.data:
            map.append(index)
        height = self.map.info.height 
        width = self.map.info.width
        # Initial Number of Zeros Check
        if print_data:
            print("Initial Count:\tZeros Unchecked = {}.".format(map.count(0)))

        # Left-Right Scan
        for rows in range(width):
            for digit in range(height):
                index = rows*height + digit
                if map[index] != 100:
                    map[index] = -1
                else:
                    break
        if print_data:
            print("After Left-Right Scan:\tZeros Unchecked = {}.".format(map.count(0)))

        # Right-Left Scan
        for rows in range(width):
            for digit in range(height):
                index = height - (rows*height + digit)
                if map[index] != 100:
                    map[index] = -1
                else:
                    break
        if print_data:
            print("After Right-Left Scan:\tZeros Unchecked = {}.".format(map.count(0)))

        # Top-Bottom Scan
        for columns in range(height):
            for digit in range(width):
                index = columns + (digit*height)
                if map[index] != 100:
                    map[index] = -1
                else:
                    break
        if print_data:
            print("After Top-Bottom Scan:\tZeros Unchecked = {}.".format(map.count(0)))

        # Bottom-Up Scan
        for columns in range(height):
            for digit in range(width):
                index = width - (columns + (digit*height))
                if map[index] != 100:
                    map[index] = -1
                else:
                    break
        if print_data:
            print("After Bottom-Up Scan:\tZeros Unchecked = {}.".format(map.count(0)))
        return (map)

    def map_is_enclosed_advanced(self, map, print_process, print_final_map):
        # For Future Reference: Need a way to detect where the wall gap is
        # This is where you incorporate the second part of the algorithm in your notebook
        index = 0
        width = self.map.info.width
        height = self.map.info.height
        last_index = height * width

        while True:
            seen_a_zero = False
            zero_touches_a_unknown = False
            # go through the data and check to see if any zero touches a -1 or the edge of the map
            for index in range(last_index):
                if map[index] == 0:
                    # Although calculating the surrounding indexes would normally reach out of bounds,
                    # the map_is_enclsed_initial function ensures that there are no zeros on the edges
                    seen_a_zero = True
                    index_right = index + 1
                    index_left  = index -1
                    index_below = index + height
                    index_above = index - height
                    # if 0 touches a -1 then turn it into a -1
                    if map[index_above] == -1 or map[index_below] == -1 or map[index_left]  == -1 or map[index_right] == -1:
                        zero_touches_a_unknown = True
                        map[index] = -1
      
            if not zero_touches_a_unknown:
                if print_final_map:
                    self.print_map_terminal(map)
                if seen_a_zero:
                    break  # Time to run Edge cases
                else:
                    return False

            #Printing of the whole process, normally not needed
            if print_process:
                self.print_map_terminal(map)

        # We only get here if we think the map is enclosed, edge cases for false positives go here
        map_is_enclosed_double_checked = self.map_is_enclosed_false_positive_edge_cases(map)
        return map_is_enclosed_double_checked

    def map_is_enclosed_false_positive_edge_cases(self, map):
        # Edge case in which small enclosures remain but they should be ignored
        # Robot is about 11 by 11 pixels large, so anything smaller than that should be discarded 
        # as there is no way that the robot is in that enclosure
        index = 0
        for points in map:
            if points == 0:
                # Move 4 pixels left and right to see if we are the center of an 11 by 11 square
                for i in range(-4, 5):
                    if map[index + i] != 0:
                        return False
            index += 1  
        

        # Passed all edge cases
        return True

    def update(self):
        #update the map so that it matches what we are getting
        rospy.wait_for_service(self.get_map_topic_name)
       
        try:
            get_map_srv = rospy.ServiceProxy(self.get_map_topic_name, GetMap)
            map_object  = GetMapRequest()
            result_map  = GetMapResponse()
            result_map  = get_map_srv(map_object).map
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
        self.map = result_map

    def print_map_grid(self):
        # Ignores all the -1's
        grid = self.map.data
        print(grid)

    def crop_map(self, print_meta_data):
        # Take the full map and crop out the rows and columns that are solely -1's
        width  = self.map.info.width
        height = self.map.info.height
        grid = self.map.data
        original_length = len(grid)
        row_cropped_grid   = []
        final_cropped_grid = []
        rows_deleted    = 0
        columns_deleted = 0
        # Crop out rows
        for rows in range(height):
            row_in_question = []
            row_is_relevant = False
            for digit in range(width):
                index = rows*width + digit
                row_in_question.append(grid[index])
                if grid[index] != -1:
                    row_is_relevant = True
            if row_is_relevant:
                for index in row_in_question:
                    row_cropped_grid.append(index)
            else:
                rows_deleted += 1
        # Update the size of the array
        height -= rows_deleted

        # Crop out columns
        for columns in range(width):
            column_in_question = []
            column_is_relevant = False
            for digit in range(height):
                index = columns + (digit*width)
                column_in_question.append(row_cropped_grid[index])
                if row_cropped_grid[index] != -1:
                    column_is_relevant = True
            if column_is_relevant:
                for index in column_in_question:
                    final_cropped_grid.append(index)
            else:
                columns_deleted += 1
        # Update size of the array
        width -= columns_deleted

        # Set values
        self.map.data = final_cropped_grid
        self.map.info.height = height
        self.map.info.width = width

        #Printing
        new_length = len(final_cropped_grid)
        if print_meta_data:
            print("\n\nCropped the old map. Heres the new map's data:")
            self.map_info_no_array()
            print("Rows Deleted: {}\t Columns Deleted: {}".format(rows_deleted, columns_deleted))
            print("Data reduction of {}% !!!".format(100 - float(new_length)/original_length * 100))
        else:
            print("\nMap Cropped")

    def map_info_no_array(self):
        # Getting values
        height = self.map.info.height
        width  = self.map.info.width
        total  = len(self.map.data)
        negative_ones = 0
        zeros = 0
        hundreds = 0
        for value in self.map.data:
            if value == -1:
                negative_ones += 1
            elif value == 0:
                zeros += 1
            elif value == 100:
                hundreds += 1
        # Printing
        print("\n\nMap Info: ")
        print("Height: {}\t Width: {}\t Total Points: {} integers.".format(height, width, total))
        print("-1's: {}\t 0's: {}\t 100's: {}".format(negative_ones, zeros, hundreds))
   
    def print_map_terminal(self, map):
        width = self.map.info.width
        height = self.map.info.height
        map_to_print_string = "\n\nCharacterized Map:\n\t"
        unknown_ch = ' '
        wall_ch    = '#'
        empty_ch   = '.'
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                if map[index] == 100:
                    map_to_print_string += wall_ch
                elif map[index] == 0:
                    map_to_print_string += empty_ch
                elif map[index] == -1:
                    map_to_print_string += unknown_ch
            map_to_print_string += '\n\t'
        print(map_to_print_string)

    def get_map_data(self):
        # Function to make getting the maps data easier for functions outside this class
        return self.map.data
    

# NOTE:
# Set the map arg to a premade map's directory
# roslaunch my_navigation slam_mapping.launch map_yaml:="/home/csrobot/stretch_ws/Maps/DuckieTown_Map.yaml"
if __name__ == "__main__":
    rospy.init_node("slam_mapping")

    # Params
    get_map_topic_name = str(rospy.get_param("~get_map_topic_name", "/static_map"))                # Topic for GetMap service
    print_crop_data = bool(rospy.get_param("~print_crop_data", True))                              # Topic for Printing Crop Data
    print_is_enclosed_data = bool(rospy.get_param("~print_is_enclosed_data", True))                # Topic for Printing the enclosed data
    print_is_enclosed_process = bool(rospy.get_param("~print_is_enclosed_process", True))          # Topic for Printing the enclosed process
    print_final_terminal_map = bool(rospy.get_param("~print_final_terminal_map", True))            # Topic for Printing final map in the terminal


    map_service = rospy.ServiceProxy(get_map_topic_name, GetMap)
    gmap = gmap(get_map_topic_name, map_service)

    gmap.update()
    gmap.map_info_no_array()

    gmap.crop_map(print_crop_data)
    gmap.print_map_terminal(gmap.get_map_data())

    print("\n\nMap_is_Enclosed: {}.".format(gmap.map_is_enclosed(print_is_enclosed_data, print_is_enclosed_process, print_final_terminal_map)))
    gmap.map_info_no_array()
