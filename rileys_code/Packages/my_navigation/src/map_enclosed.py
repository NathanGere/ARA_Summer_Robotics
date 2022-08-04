#!/usr/bin/env python
from operator import is_
import rospy
from geometry_msgs.msg  import Point
from nav_msgs.msg       import OccupancyGrid
from nav_msgs.srv       import GetMap, GetMapRequest, GetMapResponse

# Global Variables for empty, occupied, and unknown values
unknown     =   int(-1)
empty       =   int(0)
occupied    =   int(100)


class map:

    def __init__(self, get_map_topic_name):
        # General
        self.get_map_topic_name = get_map_topic_name

        # Original_Map
        self.original_map_exists = False
        self.original_map = OccupancyGrid()
        self.uncropped_indexes = []

        # Cropped Map
        self.cropped_map_exists = False
        self.cropped_map = OccupancyGrid()

        # Formatted Map
        self.formatted_map_exists = False
        self.formatted_map = OccupancyGrid()
        self.wall_gaps = []

        # Current Map
        self.current_map_exists = False
        self.current_map = OccupancyGrid()
        
        # Function calls
        self.update(first_time=True)
        self.crop_map()

    # This function takes the cropped_map or formatted_map and returns true if it is enclosed, false if it is not enclosed
    # @param is_formatted: tells the function is the map is a formatted_map or cropped_map. True=formatted_map, Flase=cropped_map
    def map_is_enclosed(self, map, is_formatted=False):
        if ( (is_formatted and self.formatted_map_exists) or (not is_formatted and self.cropped_map_exists)):
            # PART ONE
            height, width = map.info.height, map.info.width
            last_index = height * width
            # Make a shallow copy of our map 
            part_one_map = []
            for index in map.data:
                part_one_map.append(index)
            # Left-Right Scan
            for rows in range(width):
                for digit in range(height):
                    right_approach_index = rows*height + digit
                    right_approach_value = part_one_map[right_approach_index]
                    left_approach_index  = height - right_approach_index
                    left_approach_value  = part_one_map[left_approach_index]
                    right_hit_wall, left_hit_wall = False, False

                    if right_approach_value != occupied and right_hit_wall is False:
                        if right_approach_value == empty:
                            right_approach_value = unknown
                    else:
                        right_hit_wall = True
                    if left_approach_value != occupied and left_hit_wall is False:
                        if left_approach_value == empty:
                            left_approach_value = unknown
                    else:
                        left_hit_wall = True
                    if right_hit_wall and left_hit_wall:
                        break
            # Top-Bottom Scan
            for columns in range(height):
                for digit in range(width):
                    downward_approach_index = columns + (digit*height)
                    downward_aprroach_value = part_one_map[downward_approach_index]
                    upward_approach_index   = width - downward_approach_index
                    upward_approach_value   = part_one_map[upward_approach_index]
                    up_hit_wall, down_hit_wall = False, False

                    if downward_aprroach_value != occupied and down_hit_wall is False:
                        if downward_aprroach_value == empty:
                            downward_aprroach_value = unknown
                    else:
                        down_hit_wall = True
                    if upward_approach_value != occupied and up_hit_wall is False:
                        if upward_approach_value == empty:
                            upward_approach_value = unknown
                    else:
                        up_hit_wall = True
                    if up_hit_wall and down_hit_wall:
                        break
            # Optional printing to showcase the process
            if print_is_enclosed_process:
                print("After part one")
                self.print_map_terminal(part_one_map, height, width)

            # PART TWO
            while True:
                seen_an_empty = False
                empty_touches_an_unknown = False
                # go through the data and check to see if any empty touches an unknown
                for index in range(last_index):
                    if part_one_map[index] == empty:
                        # Although calculating the surrounding indexes would normally reach out of bounds,
                        # the map_is_enclosed_initial function ensures that there are no empties on the edges
                        seen_an_empty = True
                        index_right, index_left  = index + 1, index - 1
                        index_below, index_above = index + height, index - height
                        # if an empty touches an unknown then turn it into an unknown
                        if (part_one_map[index_above] == unknown or part_one_map[index_below] == unknown or 
                        part_one_map[index_left]  == unknown or part_one_map[index_right] == unknown):
                            empty_touches_an_unknown = True
                            part_one_map[index] = unknown
                if not empty_touches_an_unknown:
                    if print_final_terminal_map is True:
                        self.print_map_terminal(part_one_map, height, width)
                    if seen_an_empty is True:
                        break  # Time to run Edge cases
                    else:
                        return False
                # Printing of the whole process, normally not needed
                if print_is_enclosed_process is True:
                    self.print_map_terminal(part_one_map, height, width)
            # We only get here if we think the map is enclosed, edge cases go here
            map_is_enclosed_double_checked = self.map_is_enclosed_edge_cases(part_one_map)
            return map_is_enclosed_double_checked
        else:
            if is_formatted:
                print("map_is_enclosed failed because formatted_map does not exist and is_formatted was marked as true")
                print("Make sure to use format_map before calling map_is_enclosed with is_formatted marked as true.")
            else:
                print("map_is_enclosed failed because cropped_map does not exist: make sure crop_map is called before using map_is_enclosed.")
    
    # This function deals with edge cases from map_is_enclosed
    def map_is_enclosed_edge_cases(self, map):
        # Edge case in which small enclosures remain but they should be ignored
        # Robot is about 11 by 11 pixels large, so anything smaller than that should be discarded 
        # as there is no way that the robot is in that enclosure
        index = 0
        for points in map:
            is_a_row_of_eleven = True
            if points == empty:
                # Move 4 pixels left and right to see if we are the center of an 11 by 11 row
                for i in range(-4, 5):
                    if map[index + i] != empty:
                        is_a_row_of_eleven = False
                if is_a_row_of_eleven:
                    return True   
            index += 1  
        return False

    # This function finds wall gaps in the cropped map and populates self.wall_gaps with the indexes of those gaps
    def find_wall_gaps(self):
        if self.cropped_map_exists:
            height, width = self.cropped_map.info.height, self.cropped_map.info.width
            last_index = height * width
            # go through the data and check to see if any empty touches an unknown
            for index in range(last_index):
                if self.cropped_map.data[index] == empty:
                    index_right,  index_left   =  index + 1     ,  index - 1
                    index_below,  index_above  =  index + height,  index - height
                    # if an empty touches an unknown then its a gap in the wall
                    if (self.cropped_map.data[index_above] == unknown or 
                        self.cropped_map.data[index_below] == unknown or 
                        self.cropped_map.data[index_left]  == unknown or
                        self.cropped_map.data[index_right] == unknown):
                        self.wall_gaps.append(index)
        else:
            print("find_wall_gaps has failed: cropped_map does not exist. Make sure crop_map is called before calling this function.")

    # This function uses the GetMap service to update self.current_map parameter
    # @param first_time: optional and only used in __init__ so that the first time this function is called it populates original_map
    def update(self, first_time=False):
        rospy.wait_for_service(self.get_map_topic_name)
        try:
            get_map_srv = rospy.ServiceProxy(self.get_map_topic_name, GetMap)
            map_object  = GetMapRequest()
            result_map  = GetMapResponse()
            result_map  = get_map_srv(map_object).map
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
        if first_time:
            self.original_map = result_map
            self.original_map_exists = True
        else:
            self.current_map = result_map
            self.current_map_exists = True

    # This function crops the original_map and stores the result in self.cropped_map
    def crop_map(self):
        if self.original_map_exists:
            # Take the original_map and crop out the rows and columns that are solely unknown
            height, width = int(self.original_map.info.height), int(self.original_map.info.width)
            grid = self.original_map.data
            original_length = len(grid)
            row_cropped_grid, final_cropped_grid  = [], []
            rows_deleted, columns_deleted = 0, 0
            # Crop out rows
            for rows in range(height):
                row_in_question = []
                row_is_relevant = False
                for digit in range(width):
                    index = rows*width + digit
                    row_in_question.append(grid[index])
                    if grid[index] != unknown:
                        row_is_relevant = True
                if row_is_relevant:
                    for loop_index in row_in_question:
                        row_cropped_grid.append(loop_index)
                        self.uncropped_indexes.append(index)
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
                    if row_cropped_grid[index] != unknown:
                        column_is_relevant = True
                if column_is_relevant:
                    for loop_index in range(len(column_in_question)):
                        final_cropped_grid.append(column_in_question[loop_index])
                else:
                    columns_deleted += 1
            # Update size of the array
            width -= columns_deleted

            # Set values
            self.cropped_map.data = final_cropped_grid
            self.cropped_map.info.height, self.cropped_map.info.width = height, width
            self.cropped_map_exists = True
            #Printing
            new_length = len(final_cropped_grid)
            if print_crop_data:
                print("\n\nCropped the old map. Heres the new map's data:")
                self.map_info_no_array(self.cropped_map)
                print("Rows Deleted: {}\t Columns Deleted: {}".format(rows_deleted, columns_deleted))
                print("Data reduction of {}% !!!".format(100 - float(new_length)/original_length * 100))
            else:
                print("\nMap Cropped")
        else:
            print("crop_map has failed: original_map does not exist. Make sure __init__ is called before calling this function.")

    # This function gives basic metadata on the map given to it
    def map_info_no_array(self, map):
        # Getting values
        height = map.info.height
        width  = map.info.width
        total  = len(map.data)
        unknown_count = 0
        empty_count = 0
        occupied_count = 0
        for value in map.data:
            if value == unknown:
                unknown_count += 1
            elif value == empty:
                empty_count += 1
            elif value == occupied:
                occupied_count += 1
        # Printing
        print("\n\nMap Info: ")
        print("Height: {}\t Width: {}\t Total Points: {} integers.".format(height, width, total))
        print("Unknown: {}\t Empty: {}\t Occupied: {}".format(unknown_count, empty_count, occupied_count))
   
    # This map deals prints a version of the given map to the terminal using certain characters to represent data
    def print_map_terminal(self, map, height, width):
        unknown_ch, wall_ch, empty_ch, gap_ch = ' ', '#', '.', '0'
        map_to_print_string = "\n\nCharacterized Map:\n\t"
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                value = map[index]
                if self.wall_gaps.count(index):
                    map_to_print_string += gap_ch
                elif value == occupied:
                    map_to_print_string += wall_ch
                elif value == empty:
                    map_to_print_string += empty_ch
                elif value == unknown:
                    map_to_print_string += unknown_ch  
            map_to_print_string += '\n\t'
        print(map_to_print_string)

    # DEAL WITH This function doesnt work yet :)
    def index_to_point(self, index):
        #NOTE no idea if this works yet
        # Returns the xy coord of the index given from the cropped map
        index_for_uncropped_map = self.uncropped_indexes[index]
        print(index_for_uncropped_map)
        resolution = self.uncropped_map_info.resolution
        height = 4000 #self.uncropped_map_info.width
        width = 4000 #self.uncropped_map_info.height
        origin = self.uncropped_map_info.origin         
        xy_coord = Point()
        x_coord = origin.position.x + resolution * (index_for_uncropped_map%width)
        y_coord = origin.position.y + resolution * (index_for_uncropped_map/width)
        xy_coord.x, xy_coord.y = x_coord, y_coord
        return xy_coord

    # This function takes cropped_map and turns wall gaps into walls, storing the result in self.formatted_map
    def format_map(self):
        if self.cropped_map_exists:
            self.formatted_map.header = self.cropped_map.header
            self.formatted_map.info   = self.cropped_map.info
            formatted_map_data = []
            for data_points in self.cropped_map.data:
                formatted_map_data.append(data_points)
            for points in self.wall_gaps:
                formatted_map_data[points] = occupied
            self.wall_gaps = []
            self.formatted_map.data = formatted_map_data
            self.formatted_map_exists = True
        else:
            print("format_map has failed: cropped_map does not exist. Make sure crop_map is called before calling this function.")

    # This function publishes the given map so it can be saved as a PGM and YAML file
    def publish_map(self, map):
        pub_map = OccupancyGrid()
        pub_map.header = map.header
        pub_map.info = map.info
        map_data = []
        for index in map.data:
            map_data.append(index)
        pub_map.data = map_data
        # Width and height have to be swapped for pgm format
        pub_map.info.width, pub_map.info.height = pub_map.info.height, pub_map.info.width
        height, width = pub_map.info.height, pub_map.info.width
        # Map needs to be flipped as well for pgm format
        for row in range(height):
            # flip the row
            for index in range(width):
                start_index = row * width
                end_index = start_index + width -1
                pub_map.data[start_index + index] = map.data[end_index - index]

        closed_map_pub.publish(pub_map)
        rospy.spin()


# roslaunch my_navigation map_enclosed.launch initial_map_yaml:="/home/csrobot/stretch_ws/Maps/DuckieTown_Map.yaml" final_map_file_name:="enclosed_map"
if __name__ == "__main__":
    # Ros Node & Params
    rospy.init_node("slam_mapping")
    closed_map_pub = rospy.Publisher("/enclosed_map", OccupancyGrid, queue_size=10)
    get_map_topic_name        = str(rospy.get_param("~get_map_topic_name", "/static_map"))
    print_crop_data           = bool(rospy.get_param("~print_crop_data", True))                              
    print_is_enclosed_process = bool(rospy.get_param("~print_is_enclosed_process", True))          
    print_final_terminal_map  = bool(rospy.get_param("~print_final_terminal_map", True))    

    # All "if 0:"s below are functional programs that I dont want to run for now, but I want them saved
    if 1: 
        # Checking if map is enclosed and if not, making it enclosed
        gmap = map(get_map_topic_name)
        is_enclosed =  gmap.map_is_enclosed(gmap.cropped_map)
        print("\nMap_is_Enclosed: {}.\n".format(is_enclosed))
        if is_enclosed is not True:
            gmap.find_wall_gaps()
            print("Formatting Map")
            gmap.format_map()
            is_enclosed =  gmap.map_is_enclosed(gmap.formatted_map, is_formatted=True)
            print("\nMap_is_Enclosed: {}.\n".format(is_enclosed))
        # Publishing the map so it can be saved by map_saver
        print("Publishing Closed Map.")
        gmap.publish_map(gmap.formatted_map)
    if 0:
        # Testing index to coord
        gmap = map(get_map_topic_name)
        print(gmap.uncropped_indexes)
        print(len(gmap.uncropped_indexes))
        for i in range(0,500):
            coordinate = gmap.index_to_point(i)
            #print("Origin: [{}, {}]".format(gmap.uncropped_map_info.origin.position.x, gmap.uncropped_map_info.origin.position.y))
            print("Point {}:  [{}, {}]".format(i,coordinate.x, coordinate.y))

#TODO:
##############################################################################################################
### This file is complete. Next goal is to launch a normal mapping.launch or mapping_gazebo.launch and to  ###
### then add my own version of the keyboard teleop that replaces the old one. Instead of keyboard teleop,  ###
### the robot will autonomously move throughout the map and replace possible gaps.                         ###
##############################################################################################################
### Possible change that might be needed to this file is that instead of getting a map from the map_server ###
### service, I should get rid of the map_server in the launch file and instead solely use the GetMap       ###
### service as it is already subscribed to the map topic that rviz will provide when navigation is run     ###
##############################################################################################################

# TODO function to give xy coords from the index in the cropped map array 
# NOTE currently facing porblems with the uncropped_indexes array

### Currently woking on: