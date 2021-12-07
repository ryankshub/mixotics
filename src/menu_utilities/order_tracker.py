#!/usr/bin/env python3
#Order Tracker class file
#RKS

# Project imports
from menu_utilities.enum_defs import InstrEnum

# Python imports

# 3rd-party imports


class OrderTracker:
    """
    Responsible for tracking orders and designing instruction queue.
    """
    def __init__(self):
        """
        Constructor for ObjectTracker
        """
        # Constants
        self._NUM_CUP_SLOTS = 4
        self._MAX_INSTRS = 65535
        
        # Counters
        self._instr_counter = 1
        
        # Queues
        self._instr_queue = []
        self._backlog = []

        # Dictionaries
        self._status = {} # Tracks order's completeness
        self._avail_slots = {x+1 for x in range(self._NUM_CUP_SLOTS)}
        self._occup_slots = set()

        # Placeholder
        self._active_instr = None
        
    #### PRIVATE FUNCTION ####
    #### DO NOT USE THESE DIRECTLY ####

    def _grab_slot(self):
        """
        Grab an open slot for an order

        Returns
        int cup_id: 
            Positive id of the assigned slot. 
            If no slots are available, 0 is returned 
        """
        if len(self._avail_slots) > 0:
            rtn_id = self._avail_slots.pop()
            self._occup_slots.add(rtn_id)
        else:
            rtn_id = 0
        return rtn_id
    

    def _release_slot(self, cup_id):
        """
        Release an occupied slot upon order completion

        Args:
        int cup_id:
            Positive id of the released slot
        """
        self._occup_slots.remove(cup_id)
        self._avail_slots.add(cup_id)


    def _process_orders(self):
        """
        Processes a batch of new_orders. The process assigns order to 
        available cups, breaks down recipes into instructions, and added 
        new instructions either to active_queue or backlog
        """
        new_instr_batch = []
        # While there are open slots, process
        while len(self._avail_slots) > 0 and len(self._backlog) > 0:
            # Get top drink in log and unpack values
            drink = self._backlog.pop(0)
            order_num = drink[0]
            idx = drink[1]
            recipe = drink[2]
            # Set instr_id
            instr_id = self._instr_counter
            self._instr_counter = (self._instr_counter % self._MAX_INSTRS) + 1
            # Grab open cup slot
            cup_id = self._grab_slot()
            self._status[order_num][idx][2] = cup_id
            # Split recipe for instruction processing
            for step in recipe:
                instr = (order_num, idx, cup_id, instr_id, step)
                new_instr_batch.append(instr)
        
        # Check if safe to integrate new instrs 
        # or just append them
        modify_bool = len(self._instr_queue) > 0 and \
            (self._instr_queue[0][-1][0] == InstrEnum.GRAB or \
             self._instr_queue[0][-1][0] == InstrEnum.POUR or \
             self._instr_queue[0][-1][0] == InstrEnum.DELIVER)

        self._parse_instr_batch(new_instr_batch, modify_bool)


    def _parse_instr_batch(self, instr_batch, modify=False):
        """
        Updates internal instruction queue by integrating new batch. 

        Args:
            list of instr instr_batch:
                List of new instructions to be integrated

            bool modify: default False
                If true, the new instructions will be integrated into current 
                queue for effciency. 
                If false, new instructions will be append to the end
        Mods:
            Modifies the internal list of instructions
        """
        processed = []
        processing = []
        if modify:
            processing = self._instr_queue.copy() + instr_batch
        else:
            processing = instr_batch

        # Process instructions
        while (len(processing) > 0):
            instr = processing.pop(0)
            # Integrate Grabs
            if instr[-1][0] == InstrEnum.GRAB:
                #First, add instruction
                processed.append(instr)
                #Second Find all grabs of same type and add there pours
                grabbing = True
                temp = []
                while(len(processing) > 0):
                    if processing[0][-1] == instr:
                        grabbing = True
                        processing.pop(0)
                        continue
                    elif processing[0][-1][0] == InstrEnum.RELEASE:
                        grabbing = False

                    if grabbing:
                        processed.append(processing.pop(0))
                    else:
                        temp.append(processing.pop(0))
                processing = temp.copy()
            
            #Integrate Pours
            elif instr[-1][0] == InstrEnum.POUR:
                #First, add until you get to RELEASE
                processed.append(instr)
                while processing[0][-1][0] == InstrEnum.POUR:
                    processed.append(processing.pop(0))
                # Next craft 'GRAB' and get all pours
                # We'll use the ingredient from the RELEASE CMD
                faux_grab = (InstrEnum.GRAB, processing[0][-1][1])
                # Find all grabs of same type and add there pours
                grabbing = False
                temp = []
                while(len(processing) > 0):
                    if processing[0][-1] == faux_grab:
                        grabbing = True
                        processing.pop(0)
                        continue
                    elif processing[0][-1][0] == InstrEnum.RELEASE:
                        grabbing = False

                    if grabbing:
                        processed.append(processing.pop(0))
                    else:
                        temp.append(processing.pop(0))
                processing = temp.copy()
            
            #Integrate Releases
            elif instr[-1][0] == InstrEnum.RELEASE:
                for step in range(len(processed)-1, -1, -1):
                    if processed[step][-1][0] == InstrEnum.RELEASE:
                        # Check if instruction is the same, if so, don't add
                        if processed[step][-1] == instr[-1]:
                            break
                        else:
                            # raise hell
                            raise ValueError(f"{instr[-1][0]} follows release instead of grab")
                    elif processed[step][-1][0] == InstrEnum.GRAB:
                        # Check if instruction pairs with release
                        # If so, add release
                        if processed[step][-1][1] == instr[-1][1]:
                            processed.append(instr)
                            break
                        else:
                            # raise hell
                            raise ValueError(f"{instr[-1][0]} releases different ingredient from grabbed")
            
            #Integrate Delivery
            elif instr[-1][0] == InstrEnum.DELIVER:
                processed.append(instr)
            
            else:
                #Robot does not support this instr
                #Filter it out
                pass

        if modify:
            self._instr_queue = processed.copy()
        else:
            self._instr_queue += processed.copy()


    #### PUBLIC FUNCTIONS ####
    #### HAVE AT THEE ####

    def parse_new_orders(self, new_orders):
        """
        Adds new orders to backlog and triggers processing.

        Args:
            list of tuples (int, [drink_names], [recipes]) new_orders:
                A list of tuples where the first element is a postive order 
                number, the second elem is a list of drink names, and 
                the final elem is a corresponding list of recipes
        Mods:
            list of tuples (order_num, order_idx, recipe) backlog:
                Adds new orders to backlog
        """
        # Add new orders to backlog
        parsed_orders = []
        for order in new_orders:
            # Split tuples
            order_num = order[0]
            order_names = order[1]
            order_recipes = order[2]
            # Set up status
            self._status[order_num] = []
            for i, name in enumerate(order_names):
                # Log drink into tracker
                idx = len(self._status[order_num])
                self._status[order_num].append([False, name, 0])
                parsed_orders.append((order_num, idx, order_recipes[i]))
        self._backlog += parsed_orders
        self._process_orders()


    def get_instruction(self):
        """
        Provides an instruction

        Mods:
            Removes first element from instr_queue

        Rtn:
            tuple (order_num, idx, cup_id, instr_id, instr) instruction:
                Note: if an active instruction is being processed or 
                instruction queue is empty, None is returned

        """
    
        if (self._active_instr != None or len(self._instr_queue) == 0):
            return (None, self._active_instr != None, len(self._instr_queue) == 0)

        self._active_instr = self._instr_queue.pop(0)
        return (self._active_instr, self._active_instr != None, len(self._instr_queue) == 0)
    

    def verify_result(self, instr_id, result):
        """
        Confirms the completion of an instruction

        Args:
            int instr_id:
                Id of the instruction
            bool result:
                Whether the instruction completed successfully or not
        """
        if instr_id == self._active_instr[3]:
            idx = self._active_instr[1]
            order_num = self._active_instr[0]
            if result:
                if self._active_instr[-1][0] == InstrEnum.DELIVER:
                    self._status[order_num][idx][0] = True
            else:
                # Clean object of bad order
                tainted_order = self._status.pop(order_num)
                # Release cups
                tainted_cups_ids = [x[2] for x in tainted_order if x[2] > 0]
                for cup_id in tainted_cups_ids:
                    self._release_slot(cup_id)
                # Clear instruction queue
                self._instr_queue = [x for x in self._instr_queue if x[0] != order_num]
                # Clear backlog
                self._backlog = [x for x in self._backlog if x[0] != order_num]
            
            self._active_instr = None


    def get_cup_ids(self, order_num):
        """
        Given an order number, returns the occupied cup_slots for orders

        Args:
            int order_num:
                Id of the order
        Rtns:
            list of ints cup_ids
                a list of cup slots the order it using
        """
        order_info = self._status.get(order_num, None)
        if order_info is None:
            return []
        # We only care about cup_ids being taken, i.e. non-zero
        cup_ids = [x[2] for x in order_info if x[2] > 0]
        return cup_ids


    def is_order_complete(self, order_num=None):
        """
        Checks if order is complete. If order is complete return order_num.
        If order_num is not specfied, checks all orders and returns the 
        first found complete orders. If no order is complete, returns 0

        Args:
            int order_num: default = None
                Checking if a specific order is complete. If the order
                is complete, the order_num is returned. If not, 0 is returned
        
        Rtn:
            tuple(int, list) order_details:
                Tuple where the first element is a positive num representing 
                the completed order or 0 if no order is complete. The second 
                element is a list of cup_ids corresponding to the orders
        """
        if order_num is not None:
            # Grab status and cup_ips for order
            bools = [x[0] for x in self._status.get(order_num, [(False, 'NUL', 0)])]
            drink_names = [x[1] for x in self._status.get(order_num, [(False, 'NUL', 0)])]
            cup_ids = [x[2] for x in self._status.get(order_num, [(False, 'NUL', 0)])]
            # If all cups in order are finished
            if all(bools):
                # Clear tracker, release slots and process orders
                self._status.pop(order_num)
                for cup_id in cup_ids:
                    self._release_slot(cup_id)
                self._process_orders()
                return (order_num, drink_names, cup_ids)
        else:
            # Check each and every order
            for order in self._status.keys():
                bools = [x[0] for x in self._status.get(order, [(False, 'NUL', 0)])]
                drink_names = [x[1] for x in self._status.get(order, [(False, 'NUL', 0)])]
                cup_ids = [x[2] for x in self._status.get(order, [(False, 'NUL', 0)])]
                if all(bools):
                    self._status.pop(order)
                    for cup_id in cup_ids:
                        self._release_slot(cup_id)
                    return (order, drink_names, cup_ids)
        
        return (0, [], [])