#!/usr/bin/env python
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
        # Basic init funtion that sets the map class
        self.map = OccupancyGrid()
        self.wall_gaps = []
        self.get_map_topic_name = get_map_topic_name
        self.update()
        self.uncropped_map_info = self.map.info
        self.uncropped_indexes = []
        self.crop_map(print_meta_data=False)

    def map_is_enclosed(self, print_process, print_final_map):
        part_one_map = self.map_is_enclosed_initial()

        # Optional printing to showcase the process
        if print_process:
            self.print_map_terminal(part_one_map)

        return self.map_is_enclosed_advanced(part_one_map, print_process, print_final_map)

    def map_is_enclosed_initial(self): 
        # Make a shallow copy of our map 
        map = []
        for index in self.map.data:
            map.append(index)
        height = self.map.info.height 
        width = self.map.info.width

        # Left-Right Scan
        for rows in range(width):
            for digit in range(height):
                index = rows*height + digit
                if map[index] != occupied:
                    if map[index] == empty:
                        map[index] = unknown
                else:
                    break
       
        # Right-Left Scan
        for rows in range(width):
            for digit in range(height):
                index = height - (rows*height + digit)
                if map[index] != occupied:
                    if map[index] == empty:
                        map[index] = unknown
                else:
                    break
    
        # Top-Bottom Scan
        for columns in range(height):
            for digit in range(width):
                index = columns + (digit*height)
                if map[index] != occupied:
                    if map[index] == empty:
                        map[index] = unknown
                else:
                    break
    
        # Bottom-Up Scan
        for columns in range(height):
            for digit in range(width):
                index = width - (columns + (digit*height))
                if map[index] != occupied:
                    if map[index] == empty:
                        map[index] = unknown
                else:
                    break
     
        return (map)

    def map_is_enclosed_advanced(self, map, print_process, print_final_map):
        index = 0
        width = self.map.info.width
        height = self.map.info.height
        last_index = height * width

        while True:
            seen_an_empty = False
            empty_touches_an_unknown = False
            # go through the data and check to see if any empty touches an unknown or the edge of the map
            for index in range(last_index):
                if map[index] == empty:
                    # Although calculating the surrounding indexes would normally reach out of bounds,
                    # the map_is_enclosed_initial function ensures that there are no empties on the edges
                    seen_an_empty = True
                    index_right = index + 1
                    index_left  = index -1
                    index_below = index + height
                    index_above = index - height
                    # if an empty touches an unknown then turn it into an unknown
                    if map[index_above] == unknown or map[index_below] == unknown or map[index_left]  == unknown or map[index_right] == unknown:
                        empty_touches_an_unknown = True
                        map[index] = unknown

            if not empty_touches_an_unknown:
                if print_final_map:
                    self.print_map_terminal(map)
                if seen_an_empty:
                    break  # Time to run Edge cases
                else:
                    return False

            # Printing of the whole process, normally not needed
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
            is_a_row_of_eleven = True
            if points == empty:
                # Move 4 pixels left and right to see if we are the center of an 11 by 11 square
                for i in range(-4, 5):
                    if map[index + i] != empty:
                        is_a_row_of_eleven = False
                if is_a_row_of_eleven:
                    return True   
            index += 1  
        return False

        # Passed all edge cases
        return True

    def find_wall_gaps(self):
        index = 0
        width = self.map.info.width
        height = self.map.info.height
        last_index = height * width
        seen_an_empty = False
        empty_touches_an_unknown = False
        # go through the data and check to see if any empty touches an unknown or the edge of the map
        for index in range(last_index):
            if self.map.data[index] == empty:
                # Although calculating the surrounding indexes would normally reach out of bounds,
                # the map_is_enclosed_initial function ensures that there are no empties on the edges
                seen_an_empty = True
                index_right = index + 1
                index_left  = index - 1
                index_below = index + height
                index_above = index - height
                # if an empty touches an unknown then turn it into an unknown
                if self.map.data[index_above] == unknown or self.map.data[index_below] == unknown or self.map.data[index_left]  == unknown or self.map.data[index_right] == unknown:
                    self.wall_gaps.append(index)

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
        # Ignores all the unknowns
        grid = self.map.data
        print(grid)

    def crop_map(self, print_meta_data):
        # Take the full map and crop out the rows and columns that are solely unknown
        # After this is called, self.map is cropped
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
                if grid[index] != unknown:
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
                if row_cropped_grid[index] != unknown:
                    column_is_relevant = True
            if column_is_relevant:
                for indexx in range(len(column_in_question)):
                    final_cropped_grid.append(column_in_question[indexx])
                    self.uncropped_indexes.append(columns * width + indexx)
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
        unknown_count = 0
        empty_count = 0
        occupied_count = 0
        for value in self.map.data:
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
   
    def print_map_terminal(self, map):
        width = self.map.info.width
        height = self.map.info.height
        map_to_print_string = "\n\nCharacterized Map:\n\t"
        unknown_ch = ' '
        wall_ch    = '#'
        empty_ch   = '.'
        gap_ch     = '0'
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                if self.wall_gaps.count(index):
                    map_to_print_string += gap_ch
                elif map[index] == occupied:
                    map_to_print_string += wall_ch
                elif map[index] == empty:
                    map_to_print_string += empty_ch
                elif map[index] == unknown:
                    map_to_print_string += unknown_ch
                
            map_to_print_string += '\n\t'
        print(map_to_print_string)

    def get_map_data(self):
        # Function to make getting the maps data easier for functions outside this class
        return self.map.data

    def index_to_pose(self, index):
        # Returns the xy coord of the index given from the cropped map
        index_for_uncropped_map = self.uncropped_indexes[index]
        resolution = self.uncropped_map_info.resolution
        height = self.uncropped_map_info.width
        width = self.uncropped_map_info.height
        origin = self.uncropped_map_info.origin         
        xy_coord = Point()
        x_coord = origin.position.x + resolution * (index_for_uncropped_map%width)
        y_coord = origin.position.y + resolution * (height - (index_for_uncropped_map/width))

        #debug
        print("big_map_index: {}".format(index_for_uncropped_map))
        print("height: {}".format(height))
        print("width: {}".format(width))
        print("")



        xy_coord.x = x_coord
        xy_coord.y = y_coord
        return xy_coord

    def cut_losses_and_format_map(self):
        ###This should turn any possible wall gap into a wall and then return a new formatted map###
        for points in self.wall_gaps:
            self.map.data[points] = occupied
        self.wall_gaps = []

    def publish_closed_map(self):
        # Idk why but I have to swap width and height, its weird dont ask
        x = self.map.info.width 
        self.map.info.width = self.map.info.height
        self.map.info.height = x

        # Map needs to be flipped as well, once again dont ask
        fmap = OccupancyGrid()
        fmap.header = self.map.header
        fmap.info = self.map.info
        map_data = []
        for index in self.map.data:
            map_data.append(index)
        fmap.data = map_data

        # For each row
        for row in range(fmap.info.height):
            # flip the row
            for index in range(fmap.info.width):
                start_index = row * fmap.info.width
                end_index = row * fmap.info.width + fmap.info.width -1
                fmap.data[start_index + index] = self.map.data[end_index - index]

        closed_map_pub.publish(fmap)
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
    if 0: 
        # Checking if map is enclosed and if not, making it enclosed
        gmap = map(get_map_topic_name)
        is_enclosed =  gmap.map_is_enclosed(print_is_enclosed_process, print_final_terminal_map)
        print("\nMap_is_Enclosed: {}.\n".format(is_enclosed))
        if is_enclosed is not True:
            gmap.find_wall_gaps()
            print("Formatting Map")
            gmap.cut_losses_and_format_map()
            is_enclosed =  gmap.map_is_enclosed(print_is_enclosed_process, print_final_terminal_map)
            print("\nMap_is_Enclosed: {}.\n".format(is_enclosed))


        # Publishing the map so it can be saved by map_saver
        #print("Publishing Closed Map.")
        #gmap.publish_closed_map()
    if 1:
        gmap = map(get_map_topic_name)
        coordinate = gmap.index_to_pose(32456)
        print("Origin: [{}, {}]".format(gmap.uncropped_map_info.origin.position.x, gmap.uncropped_map_info.origin.position.y))
        print("Point:  [{}, {}]".format(coordinate.x, coordinate.y))

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
### Odd thing to note: height and width need to be swapped in this program before working with data points ###
### and once again when converting back to an occupancy grid message to publish                            ###
##############################################################################################################


# function to give xy coords from the index in the cropped map array 


