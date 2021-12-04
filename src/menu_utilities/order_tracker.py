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
            order = self._backlog.pop(0)
            self._status[order[0]] = []
            for recipe in order[1]:
                idx = len(self._status[order[0]])
                instr_id = self._instr_counter
                self._instr_counter = (self._instr_counter % self._MAX_INSTR) + 1
                cup_id = self._grab_slot()
                self._status[order[0]].append((False, cup_id))
                for step in recipe:
                    instr = (order[0], idx, cup_id, instr_id, step)
                    new_instr_batch.append(instr)
        
        # Check if safe to integrate new instrs 
        # or just append them
        modify_bool = False
        if len(self._instr_queue) > 0 and \
            (self._instr_queue[0][-1][0] == InstrEnum.GRAB or \
             self._instr_queue[0][-1][0] == InstrEnum.POUR or \
             self._instr_queue[0][-1][0] == InstrEnum.DELIVERY):
            modify_bool = True

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
                for step in range(0, len(processed), -1):
                    if step[-1][0] == InstrEnum.RELEASE:
                        # Check if instruction is the same, if so, don't add
                        if step[-1] == instr[-1]:
                            break
                        else:
                            # raise hell
                            raise ValueError(f"{instr[-1][0]} follows release instead of grab")
                    elif step[-1][0] == InstrEnum.GRAB:
                        # Check if instruction pairs with release
                        # If so, add release
                        if step[-1][1] == instr[-1][1]:
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
            list of tuples (int, [recipes]) new_orders:
                A list of tuples where the first element is a postive order 
                number and the second elem is a list of recipes
        """
        # Add new orders to backlog
        self._backlog += new_orders
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
            return None

        self._active_instr = self._instr_queue.pop(0)
        return self._active_instr
    

    def verify_result(self, instr_id, result):
        """
        Confirms the completion of an instruction

        Args:
            int instr_id:
                Id of the instruction
            bool result:
                Whether the instruction completed successfully or not
        """
        if instr_id == self._active_instr[4]:
            idx = self._active_instr[1]
            order_num = self._active_instr[0]
            if result and self._active_instr[-1][0] == InstrEnum.DELIVER:
                self._status[order_num][idx][0] = True
            else:
                # Clean object of bad order
                self._status.pop(order_num)
                self._instr_queue = [x for x in self._instr_queue if x[0] != order_num]
                self._backlog = [x for x in self._backlog if x[0] != order_num]


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
            bools = [x[0] for x in self._status.get(order_num, [(False, 0)])]
            cup_ids = [x[1] for x in self._status.get(order_num, [(False, 0)])]
            if all(bools):
                self._status.pop(order_num)
                for cup_id in cup_ids:
                    self._release_slot(cup_id)
                return (order_num, cup_ids)
        else:
            for order in self._status.keys():
                bools = [x[0] for x in self._status.get(order_num, [(False, 0)])]
                cup_ids = [x[1] for x in self._status.get(order_num, [(False, 0)])]
                if all(bools):
                    self._status.pop(order_num)
                    for cup_id in cup_ids:
                        self._release_slot(cup_id)
                    return (order_num, cup_ids)