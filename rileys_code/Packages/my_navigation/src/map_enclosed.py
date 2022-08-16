#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Point
from nav_msgs.msg       import OccupancyGrid
from nav_msgs.srv       import GetMap, GetMapRequest, GetMapResponse


# Global Variables for empty, occupied, and unknown values
unknown     =   int(-1)
empty       =   int(0)
occupied    =   int(100)


class room:

    def __init__(self, empty_indexes, name="default_name"):
        self.name = name
        self.empty_indexes = empty_indexes
        ##self.home_index = home_index (possible future project)

    def change_name(self, new_name):
        self.name = new_name


class map:

    def __init__(self, get_map_topic_name):
        # General
        self.get_map_topic_name = get_map_topic_name

        # Rooms
        self.rooms = []
        self.room_count = 1

        # FULLSIZE MAPS:
        # Original_Map
        self.original_map_exists = False
        self.original_map = OccupancyGrid()

        # Current Map
        self.current_map_exists = False
        self.current_map = OccupancyGrid()
        self.differences_from_original= []

        # COMPACT MAPS:
        # Cropped Map
        self.cropped_map_exists = False
        self.cropped_map = OccupancyGrid()
        self.bottom_left_point = Point()

        # Formatted Map
        self.formatted_map_exists = False
        self.formatted_map = OccupancyGrid()
        self.wall_gaps = []

        # Minimalist_map
        self.minimalist_map_exists = False
        self.minimalist_map = OccupancyGrid()
        self.object_indexes = []
        
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

    # This function crops the original_map/current_map and stores the result in self.cropped_map
    def crop_map(self, storing_map=True, current_map=False):
        # Choosing which map to crop
        if self.original_map_exists:
            if current_map:
                if self.current_map_exists:
                    map_to_crop = OccupancyGrid()
                    self.copy_map(map_to_copy=self.current_map, map_to_copy_to=map_to_crop)
                else:
                    print("Cannot crop current_map because it does not exist. Call update() first") 
            else:   
                map_to_crop = OccupancyGrid()
                self.copy_map(map_to_copy=self.original_map, map_to_copy_into=map_to_crop)
            # Take the original_map and crop out the rows and columns that are solely unknown
            height, width = int(map_to_crop.info.height), int(map_to_crop.info.width)
            grid = map_to_crop.data
            original_length = len(grid)
            row_cropped_grid, final_cropped_grid  = [], []
            rows_deleted, columns_deleted = 0, 0
            first_accepted_row, first_accepted_column = True, True
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
                    if first_accepted_row and storing_map:
                        self.bottom_left_point.y = map_to_crop.info.height - rows
                    for loop_index in row_in_question:
                        row_cropped_grid.append(loop_index)
                    first_accepted_row = False
                else:
                    rows_deleted += 1
            # Update size of the array
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
                    if first_accepted_column and storing_map:
                        self.bottom_left_point.x = columns
                    for loop_index in range(len(column_in_question)):
                        final_cropped_grid.append(column_in_question[loop_index])
                    first_accepted_column = False
                else:
                    columns_deleted += 1
             # Update size of the array
            width -= columns_deleted

            if storing_map:
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
                # Return the map
                map_to_return = OccupancyGrid()
                map_to_return.data = final_cropped_grid
                map_to_return.info.height, map_to_return.info.width = height, width
                return map_to_return
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
   
    # This function prints a version of the given map to the terminal using certain characters to represent data
    def print_map_terminal(self, map):
        height, width = map.info.height, map.info.width
        unknown_ch, wall_ch, empty_ch, gap_ch = ' ', '#', '.', '0'
        map_to_print_string = "\n\nCharacterized Map:\n\t"
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                value = map.data[index]
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

    # This function returns the xy coord of the index given from the cropped maps (anything but original and current)
    def index_to_point(self, map, index):
        top_left_index = self.bottom_left_point
        
        xy_coord = Point()
        x_coord = top_left_index.x + (index%height)
        y_coord = top_left_index.y - (index/map.info.width)
     
        # Account for the resolution of pixels to meters
        x_coord =  float(x_coord) * self.original_map.info.resolution
        y_coord =  float(y_coord) * self.original_map.info.resolution

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

    # This function allows the user to choose between the original & current map, and the choice will be copied to both maps
    def map_repair(self): 
        #map_prestige 
        if self.current_map_exists and self.original_map_exists:
            # As of now, this assumes that both maps are of the same size
            current_map_size, original_map_size = len(self.current_map.data), len(self.original_map.data)
            if current_map_size != original_map_size:
                print("These maps are of different sizes, comparing them does not work.")
                return
            # if a certain index is different between maps, save that index to a designated list
            maps_are_different = False
            for index in range(len(self.original_map.data)):
                if self.original_map.data[index] != self.current_map.data[index]:
                    maps_are_different = True
                    self.differences_from_original.append(index)
            if maps_are_different is False:
                print("Current map is identical to Original map. No repairs needed.")
            else:
                print("There are {} differences between Current map and the Original.".format(len(self.differences_from_original)))
                response_one = raw_input("Would you like to see the differences between the maps before choosing which one to trust? (y/n): ")
                if response_one.lower() == 'y':
                    print("Sure thing!! Heres the original map: ")
                    original_map = self.crop_map(storing_map=False)
                    self.map_info_no_array(original_map)
                    self.print_map_terminal(original_map)

                    print("\n\nAnd here is the current map: ")
                    current_map = self.crop_map(storing_map=False, current_map=True)
                    self.map_info_no_array(current_map)
                    self.print_map_terminal(current_map)

                print("\n\nOkay, which map would you like to trust?")
                print("(The trusted map will be copied to the other map)")
                response_two = raw_input('Press "1" to trust the original map, "2" to trust the current map, or anything else to quit: ')
                if response_two == '1':
                    print("Copying original map to current map.")
                    self.current_map = self.original_map
                elif response_two == '2':
                    print("Copying current map to original map.")
                    self.original_map = self.current_map
                else:
                    print('Quitting')
        else:
            if not self.current_map_exists:
                print("map_repair failed because current_map does not exist: make sure update is called before calling map_repair")
            elif not self.original_map_exists:
                print("map_repair failed because original_map does not exist: make sure __init__ is called before calling map_repair")

    # This function autonomously removes all objects in the map leaving only walls and stores it to minimalist_map
    # POSSIBLE ERRORS will remove kitchen islands and island walls, as well as poles
    # another slight error is this map will read as not enclosed when in reality it is enclosed
    def remove_furniture(self):
        #formatted_map is needed
        if self.formatted_map_exists:
            # save the indexes of all the occupied points in the formatted map
            height= self.formatted_map.info.height
            occupied_points = []
            for indx in range(len(self.formatted_map.data)):
                if self.formatted_map.data[indx] == occupied:
                    occupied_points.append(indx)
            # remove all the exterior wall points from the occupied points
            to_remove = []
            already_removed = []
            to_remove.append(occupied_points[0])
            while len(to_remove) > 0:
                #find all the walls that border the first index
                index = to_remove[0]
                index_east,  index_west             =  index + 1     ,  index - 1
                index_south,  index_north           =  index + height,  index - height
                index_southeast, index_southwest    =  index_south + 1, index_south -1
                index_northeast, index_northwest    =  index_north + 1, index_north -1
                surrounding_indexes = ([index_northwest, index_north, index_northeast,
                                        index_west,                        index_east, 
                                        index_southwest, index_south, index_southeast])
                # If they are occupied and havent been removed yet, add them to the queue
                for indexes in surrounding_indexes:
                    if ((indexes in occupied_points) and (not indexes in already_removed)):
                        to_remove.append(indexes)
                        occupied_points.remove(indexes)
                # Remove the index we started with
                to_remove.remove(index)
                already_removed.append(index)
            print("Number of points I think are objects: {}".format(len(occupied_points)))
            #########Waiting for the copy_map function, for now ill just make the changes directly to formatted_map##########
            self.copy_map(map_to_copy=self.formatted_map, map_to_copy_into=self.minimalist_map)
            self.minimalist_map_exists = True
            print("minimal:")
            self.map_info_no_array(self.minimalist_map)
            print("Formatted:")
            self.map_info_no_array(self.formatted_map)
            for points in occupied_points:
                self.minimalist_map.data[points] = empty
            # Theres a case in which some of the space in our enclosed map is now unknown
            print("Minimalist map:")
            self.print_map_terminal(self.minimalist_map)
        else:
            print("remove_objects has failed: formatted_map does not exist. Make sure format_map is called before calling this function.")
    
    # This function DEEP copies one map to the other
    def copy_map(self, map_to_copy, map_to_copy_into):
        # info
        map_to_copy_into.info.height, map_to_copy_into.info.width = map_to_copy.info.height, map_to_copy.info.width
        for points in map_to_copy.data:
            map_to_copy_into.data.append(points)

    ######################### AFTER THIS LINE WE ARE WORKING WITH MAP SECTORS AND ROOMS #################################

    # This function should create room objects for all the user defined rooms in a map
    def make_rooms(self):
        seperated_room_map = self.mark_rooms()
        num_rooms = self.room_count
        print("Number of Rooms: {}".format(num_rooms))

        print("Creating rooms...")
        empty_points = []
        height = seperated_room_map.info.height
        for index in range(len(seperated_room_map.data)):
            if seperated_room_map.data[index] == empty:
                empty_points.append(index)

        created_rooms = 0
        while len(empty_points) > 0:   
            # remove all the exterior wall points from the occupied points
            to_add_to_room = []
            already_added = []
            to_add_to_room.append(empty_points[0])
            while len(to_add_to_room) > 0:
                #find all the walls that border the first index
                index = to_add_to_room[0]
                index_east,  index_west             =  index + 1     ,  index - 1
                index_south,  index_north           =  index + height,  index - height
                surrounding_indexes = [ index_north, index_west, index_east, index_south,]
                # If they are occupied and havent been removed yet, add them to the queue
                for indexes in surrounding_indexes:
                    if ((indexes in empty_points) and (not indexes in already_added)):
                        to_add_to_room.append(indexes)
                        empty_points.remove(indexes)
                # Remove the index we started with
                if index in empty_points:
                    empty_points.remove(index)
                to_add_to_room.remove(index)
                already_added.append(index)

            # if the number of empties in a room is less than 100 then dont count it as a room
            # less than 100 points ownt even fit the robot so thats clearly an error
            if len(already_added) > 100:
                self.rooms.append(room(already_added))
                created_rooms += 1
                print("Room created.")
        print("Finished, {} rooms made.".format(created_rooms))

        # Edge Case where too many rooms are created
        too_many_rooms_created = False 
        while created_rooms > int(num_rooms):
            too_many_rooms_created = True
            print("Oops. You only asked for {} rooms but I created {} valid rooms.".format(num_rooms, created_rooms))
            print("I'll show you all the rooms and you tell me if its one of the rooms you wanted me to create.")
            print("(You need to remove {} rooms)".format(created_rooms-num_rooms))
            raw_input("Press enter to continue")
            subtract_from_created_rooms = 0
            self.rooms_to_remove = []
            for map_room in self.rooms:
                self.print_room(map_room)
                # ask if they want it to be a room (y/n)
                choice = raw_input("Would you like this to be a room?(y/n)")
                if choice is 'y':
                    room_name = raw_input("What would you like to name this room?\n> ")
                    map_room.change_name(room_name)
                else:
                    self.rooms_to_remove.append(map_room)
                    subtract_from_created_rooms += 1
            for rooms in self.rooms_to_remove:
                self.rooms.remove(rooms)
            created_rooms -= subtract_from_created_rooms
        self.room_count = created_rooms
        # Ask user to name each room
        if not too_many_rooms_created:
            for map_room in self.rooms:
                self.print_room(map_room)
                room_name = raw_input("What would you like to name this room?\n> ")
                map_room.change_name(room_name)

    # This function prints a room to the terminal on the cropped_map
    def print_room(self, room):
        height, width = self.minimalist_map.info.height, self.minimalist_map.info.width
        unknown_ch, wall_ch, empty_ch = ' ', '#', '.'
        map_to_print_string = "\n\nCharacterized Map:\n\t"
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                value = self.minimalist_map.data[index]
                if room.empty_indexes.count(index):
                    map_to_print_string += empty_ch
                elif value == occupied:
                    map_to_print_string += wall_ch
                elif value == empty or value == unknown:
                    map_to_print_string += unknown_ch 
            map_to_print_string += '\n\t'
        print(map_to_print_string)

    # This function lists the names of all the rooms
    def list_rooms(self):
        print("{} rooms:".format(len(self.rooms)))
        for map_room in self.rooms:
            print("\t{}".format(map_room.name))

    # This function takes the formatted map and asks the user to seperate the map into rooms.
    # then the rooms are enclosed with new walls. This function should be paired with self.make_rooms
    def mark_rooms(self):
        print("Welcome! Lets take this map and seperate it by rooms.")
        raw_input("Press enter to continue.\n> ")

        # Deep copy the minimalist_map
        temp_map = OccupancyGrid()
        self.copy_map(map_to_copy=self.minimalist_map, map_to_copy_into=temp_map)
        height = temp_map.info.height
        width = temp_map.info.width
        possible_corners_indexes = []
        chosen_corners_indexes = []
        self.get_possible_corners(temp_map, possible_corners_indexes)
        # Ask the user which points they like for possible corners
        print("\nI have found {} possible points to seperate rooms by.\n". format(len(possible_corners_indexes)))
        print("Please tell me which points are valid by typing the corners by the digits they are marked by and then hitting [enter]")
        print('As an example, if points 3, 7, and 9 are valid, you would type "379[enter]".')
        print("(valid points are ones that are at the edge of an entrance/exit of a possible room)")
        raw_input("Press enter to continue.\n> ")

        times_to_loop = len(possible_corners_indexes) / 10
        if len(possible_corners_indexes) % 10 != 0:
            times_to_loop += 1
        
        for i in range(times_to_loop):
            # Get the ten points to display
            ten_points_to_show = []
            for j in range(10):
                if j + i*10 < len(possible_corners_indexes):
                    ten_points_to_show.append(possible_corners_indexes[j + i*10])
            # print the map
            width = self.minimalist_map.info.width
            unknown_ch, wall_ch = ' ', '.'
            map_to_print_string = "\n\nCharacterized Map:\n\t"
            for row in range(width):
                for digit in range(height):
                    index = row * height + digit
                    value = temp_map.data[index]
                    if ten_points_to_show.count(index):
                        map_to_print_string += str(ten_points_to_show.index(index))
                    elif value == occupied:
                        map_to_print_string += wall_ch
                    elif value == empty:
                        map_to_print_string += unknown_ch
                    elif value == unknown:
                        map_to_print_string += unknown_ch  
                map_to_print_string += '\n\t'
            print(map_to_print_string)

            # Get the users input
            user_choices = raw_input("Which points are valid:\n> ")
            digits_as_strings = ['0', '1', '2', '3', '4', '5','6', '7', '8', '9']
            for digits in digits_as_strings:
                if digits in user_choices:
                    chosen_corners_indexes.append(possible_corners_indexes[int(digits) + i*10])
        print("All done with choosing corners, now lets connect them.")
        raw_input("Press enter to continue.\n> ")

        # Print the map with all the user chosen points
        characters_to_represent_corners = (['0','1','2','3','4','5','6','7','8','9',
        'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S',
        'T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i','j','k','l',
        'm','n','o','p','q','r','s','t','u','v','w','x','y','z',])
        width = self.minimalist_map.info.width 
        unknown_ch, wall_ch = ' ', '.'
        map_to_print_string = "\n\nHere is the map with the points youve chosen:\n\t"
        corners_seen = 0
        for row in range(width):
            for digit in range(height):
                index = row * height + digit
                value = temp_map.data[index]
                if chosen_corners_indexes.count(index):
                    map_to_print_string += characters_to_represent_corners[corners_seen]
                    corners_seen += 1   
                elif value == occupied:
                    map_to_print_string += wall_ch
                elif value == empty:
                    map_to_print_string += unknown_ch
                elif value == unknown:
                    map_to_print_string += unknown_ch  
            map_to_print_string += '\n\t'
        print(map_to_print_string)

        # Ask the user which points they would like to connect
        if corners_seen <= 62:
            print("What points would you like connected to form a wall that will seperate rooms?")
            if corners_seen <= 10:
                # only digits
                print("Please answer in this format: 13 46 91 38[enter]")
                print("That example would connect points 1+3, 4+6, 9+1, and 3+8")
            elif corners_seen <= 36:
                # digits and uppercase
                print("Please answer in this format: 13 46 AD 3C[enter]")
                print("That example would connect points 1+3, 4+6, A+D, and 3+C")
            elif corners_seen <= 62:
                # digits, upper and lower case
                print("Please answer in this format: 13 AD bc 3C 6b[enter]")
                print("That example would connect points 1+3, A+D, b+c, 3+C, and 6+b")
            # Connect points
            user_connections = raw_input("> ")
            for i in range(0, len(user_connections), 3):
                if characters_to_represent_corners.index(user_connections[i]) + 1:
                    first_point = chosen_corners_indexes[characters_to_represent_corners.index(user_connections[i])]
                    second_point = chosen_corners_indexes[characters_to_represent_corners.index(user_connections[i+1])]
                    self.draw_line_between_points(temp_map, first_point, second_point)
                    self.room_count += 1
        else:
            # Too many trip an error
            print("Sorry, but there are to many points for me to display with unique characters.")
            print("Please try again and choose 62 points or less.")

        return temp_map

    # This function takes two points and a map, and it draws a line between the two points with a wall
    def draw_line_between_points(self, map, point_1, point_2):
        height = map.info.height
        # make sure point 1 is below 2
        if point_1 < point_2:
            #swap them
            point_1, point_2 = point_2, point_1
        # Calculate original slope and what direction to move in (up is already known)
        original_slope = self.calculate_slope_ish(point_1, point_2, map)
        if original_slope == -13371337: # Special value where a horizontal or vertical line was automatically drawn
            return

        if original_slope > 0:
            horizontal_movement = -1
            if original_slope < 0.5:
                default_movement = horizontal_movement
            else:
                default_movement = -1 * height
        else:
            horizontal_movement = 1
            if original_slope > -0.5:
                default_movement = horizontal_movement
            else:
                default_movement = -1 * height

        # move up once for a base new_slope value
        point_1 -= height
        map.data[point_1] = occupied

        new_slope = self.calculate_slope_ish(point_1, point_2, map)
        if new_slope == -13371337: # Special value where a horizontal or vertical line was automatically drawn
            return

        while point_1 != point_2:
            if abs(new_slope) > abs(original_slope):
                point_1 -= height
                map.data[point_1] = occupied
            
                new_slope = self.calculate_slope_ish(point_1, point_2, map)
                if new_slope == -13371337: # Special value where a horizontal or vertical line was automatically drawn
                    return
            elif abs(new_slope) < abs(original_slope):
                point_1 -= horizontal_movement
                map.data[point_1] = occupied

                new_slope = self.calculate_slope_ish(point_1, point_2, map)
                if new_slope == -13371337: # Special value where a horizontal or vertical line was automatically drawn
                    return
            else:
                point_1 += default_movement
                map.data[point_1] = occupied

                new_slope = self.calculate_slope_ish(point_1, point_2, map)
                if new_slope == -13371337: # Special value where a horizontal or vertical line was automatically drawn
                    return

    # This function calculates slope and moves the drawn wall accordingly if slope is 0 or inf
    def calculate_slope_ish(self, point_1, point_2, map):
         ### get the xy coords of the points given
        height = map.info.height
        top_left_index = self.bottom_left_point
        # Point 1 to xy coords
        point1_coords =  Point()
        x_coord = top_left_index.x + (point_1%height)
        y_coord = top_left_index.y - (point_1/map.info.width)
        point1_coords.x, point1_coords.y = x_coord, y_coord
         # Point 2 to xy coords
        point2_coords =  Point()
        x_coord = top_left_index.x + (point_2%height)
        y_coord = top_left_index.y - (point_2/map.info.width)
        point2_coords.x, point2_coords.y = x_coord, y_coord
        #Calculate Slope
        rise = point2_coords.y - point1_coords.y
        run = point2_coords.x - point1_coords.x
        if run == 0.00:
            # Move up until 1 == 2 occupying indexes
            while point_1 != point_2:
                point_1 -= height
                map.data[point_1] = occupied
                return -13371337
        elif rise == 0.00:
            if point2_coords.x > point1_coords.x:
                # Move right until 1 == 2 occupying indexes
                while point_1 != point_2:
                    point_1 += 1
                    map.data[point_1] = occupied
                    return -13371337
            else:
                # Move left until 1 == 2 occupying indexes
                while point_1 != point_2:
                    point_1 -= 1
                    map.data[point_1] = occupied
                    return -13371337
        else:
            return float(rise)/run

    # This finds possible room corners and puts thier indexes into possible_corner_indexes
    def get_possible_corners(self, temp_map, possible_corners_indexes):
        height = temp_map.info.height
        # Get all walls
        walls_indexes = []
        for points in range(len(temp_map.data)):
            if temp_map.data[points] == occupied:
                walls_indexes.append(points)
        # Check each walls surrounding points
        for index in walls_indexes:
            index_east,  index_west             =  index + 1     ,  index - 1
            index_south,  index_north           =  index + height,  index - height
            index_southeast, index_southwest    =  index_south + 1, index_south -1
            index_northeast, index_northwest    =  index_north + 1, index_north -1
            surrounding_indexes = ([index_northwest, index_north, index_northeast,
                                    index_west,                        index_east, 
                                    index_southwest, index_south, index_southeast])
            num_occupied, num_empty, num_unknown = 0,0,0
            passed_line_of_10_test = False
            for neighbors in surrounding_indexes:
                if neighbors < len(temp_map.data):
                    if temp_map.data[neighbors] is occupied:
                        num_occupied += 1
                    elif temp_map.data[neighbors] is unknown:
                        num_unknown += 1
                    elif temp_map.data[neighbors] is empty:
                        num_empty += 1
            # if there are more empties than walls and there are 1 or less unknowns then its a probable corner
            if (num_empty > 2) and (num_empty > num_occupied + 1) and (num_unknown == 0) and (num_occupied <= 4):

                # Line of 20 test (can i draw an x or a + through the point and have a line that doesnt touch a wall)
                if temp_map.data[index_east] == empty and temp_map.data[index_west] == empty:
                    for i in range(-5,6):
                        if temp_map.data[index + i] != empty:
                            if i != 0:
                                break
                        if i == 5:
                            passed_line_of_10_test = True
                    
                elif  temp_map.data[index_north] == empty and  temp_map.data[index_south] == empty and passed_line_of_10_test == False:
                    for i in range(-5,6):
                        if temp_map.data[index + (height*i)] != empty:
                            if i != 0:
                                break
                        if i == 5:
                            passed_line_of_10_test = True
                elif  temp_map.data[index_northeast] == empty and  temp_map.data[index_southwest] == empty and passed_line_of_10_test == False:
                    for i in range(-5,6):
                        if temp_map.data[index + (height*i) - i] != empty:
                            if i != 0:
                                break
                        if i == 5:
                            passed_line_of_10_test = True
                elif  temp_map.data[index_northwest] == empty and  temp_map.data[index_southeast] == empty and passed_line_of_10_test == False:
                    for i in range(-5,6):
                        if temp_map.data[index + (height*i) + i] != empty:
                            if i != 0:
                                break
                        if i == 5:
                            passed_line_of_10_test = True

                if passed_line_of_10_test == True:
                    possible_corners_indexes.append(index)
                    
        # If 2 corners touch, keep the one that touches the most empties and discard the other one
        for index in possible_corners_indexes:
            index_east,  index_west             =  index + 1     ,  index - 1
            index_south,  index_north           =  index + height,  index - height
            index_southeast, index_southwest    =  index_south + 1, index_south -1
            index_northeast, index_northwest    =  index_north + 1, index_north -1
            surrounding_indexes = ([index_northwest, index_north, index_northeast,
                                    index_west,                        index_east, 
                                    index_southwest, index_south, index_southeast])
            original_empty_count, other_empty_count = 0,0
            for point in surrounding_indexes:
                # get num_empty for original point
                if temp_map.data[point] == empty:
                    original_empty_count += 1
                if point in possible_corners_indexes:
                    # get num empty for new point
                    surrounding_points = ([point-height-1, point-height, point-height+1,
                                            point-1,                      point+1, 
                                            point+height-1, point+height, point+height+1])
                    for i in surrounding_points:
                        if i == empty:
                            other_empty_count += 1
                    if other_empty_count > original_empty_count:
                        possible_corners_indexes.remove(index)
                    else:
                        possible_corners_indexes.remove(point)
       
    # This function returns the name of the room that a given index is in, or returns "no known room"
    def get_room_from_index(self, index):
        for map_room in self.rooms:
            if index in map_room.empty_indexes:
                return map_room.name
        return "This index is not under a known room"


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
    
    # Checking if map is enclosed and if not, making it enclosed
    if 0: 
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
    
    # Testing index to coord
    if 0:
        index_to_choose = 300
        print("Index {}:".format(index_to_choose))
        print(gmap.index_to_point(index_to_choose))

        print("Width:")
        print(gmap.cropped_map.info.width)     
    
    # Testing map_repair
    if 0:
        gmap = map(get_map_topic_name)
        gmap.update()
        gmap.map_repair()
    
    # Remove_furniture test
    if 0:
        gmap = map(get_map_topic_name)
        gmap.find_wall_gaps()
        print("Formatting Map")
        gmap.format_map()
        print("Format Map info")
        gmap.map_info_no_array(gmap.formatted_map)
        gmap.remove_furniture()
        gmap.print_map_terminal(gmap.formatted_map)
        gmap.publish_map(gmap.formatted_map)
    
    # Mark_rooms test
    if 1:
        gmap = map(get_map_topic_name)
        gmap.find_wall_gaps()
        gmap.format_map()
        gmap.remove_furniture()
        gmap.make_rooms()
        print(gmap.get_room_from_index(17430))
        input = "default_value"
        while input is not 'q':
            print("Here are the rooms: ")
            gmap.list_rooms()

            input = raw_input("Would you like to see any of these rooms individually? (y/n) ")
            if input == 'y':
                input = raw_input("Please type the name of the room you would like to see\n> ")
                for map_room in gmap.rooms:
                    if map_room.name == input:
                        gmap.print_room(map_room)

            input = raw_input("Would you like to rename any of these rooms? (y/n) ")
            if input == 'y':
                input = raw_input("Please type the name of the room you would like to rename\n> ")
                for map_room in gmap.rooms:
                    if map_room.name == input:
                        input = raw_input("What do you want the new name to be?\n> ")
                        map_room.change_name(input)

            input = raw_input("To quit press q and enter, else just hit enter\n> ")

#TODO:
##############################################################################################################
### Possible change that might be needed to this file is that instead of getting a map from the map_server ###
### service, I should get rid of the map_server in the launch file and instead solely use the GetMap       ###
### service as it is already subscribed to the map topic that rviz will provide when navigation is run     ###
##############################################################################################################