# UWARG High Level Analyzer
from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting
from pathlib import Path

import io
import sys
import os
import json

from pathlib import Path

lib_path = str(Path(__file__).parent / "lib")
if lib_path not in sys.path:
    sys.path.append(lib_path)

import dronecan

# Assuming ArduPilotMega Dialect
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# Create a JSON Lookup on Runtime, access it on each get_message_name call
_LOOKUP = {}
try:
    base_dir = os.path.dirname(__file__)
    json_path = os.path.join(base_dir, 'dsdl_lookup.json')

    with open(json_path, 'r') as f:
        _LOOKUP = json.load(f)

except Exception as e:
    print("Failed to load DSDL lookup:", e)

def get_message_name(target_number):
        return _LOOKUP.get(str(target_number), "Unknown Message")

def reverse_bits_16bit(x):
    result = 0
    for i in range(16):
        if (x >> i) & 1:
            result |= 1 << (15 - i)
    return result

class Hla(HighLevelAnalyzer):
    protocol_mode = ChoicesSetting(
        ['DroneCAN', 'MAVLink1', 'MAVLink2'],  
        label='Protocol'          
    )

    result_types = {
    'Full-Frame':{
        'format': 'Full Msg, Name: {{data.Name}}, Data: {{data.Data}}'
        },
    }
    

    def __init__(self):
        self.number_of_data_frames = 0
        self.numb_msgs = None
        self.current_full_msg = None
        self.frame_start = None
        self.frame_end = None
        self.message_start = None
        self.message_end = None
        self.current_end = None
        self.last_message = True
        self.multi_message = None
        self.started = False
        self.name = None
        self.data = []
        self.frames = []
        self.id = None
        self.mavlink_buf = b''
        self.mavlink_started = False

    def decode_dronecan(self, frame: AnalyzerFrame):
        if frame.type == 'identifier_field':
            #print("checking if extended")
            if 'extended' in frame.data and frame.data['extended']:
                if(self.last_message):
                    self.frames = []
                    self.last_message = False
                if(self.id != frame.data['identifier']):
                    self.frames =[]
                self.data = []
                self.frame_start = frame.start_time
                self.number_of_data_frames = 0
                value = frame.data['identifier']
                self.id = value
                # Reads the first 5 bits of the identifier
                current_service = value >> 7 & 0x01       
                # Reads the service not bit 
                current_source_node_id = value & 0x7F      
                current_priority = value >> 24 & 0x1F  
                
                if not current_service and current_source_node_id:
                    current_message_data = value >> 8 & 0xFFFF
                    message_name = get_message_name(current_message_data)
                    
                    current_message_type = 'Standard Message'

                elif current_service and current_source_node_id:
                    current_message_data = value >> 16 & 0xFF
                    current_destination_node_id = value >> 8 & 0x7F
                    message_name = get_message_name(current_message_data)
                    if value >> 15 & 0x01:
                        current_message_type = 'Request Message'

                    else:
                        current_message_type = 'Response Message'

                elif not current_service and not current_source_node_id:
                    # Reads the data type id, which is the next 7 bits after the service not bit
                    current_message_data = value >> 8 & 0x3
                    current_message_type = 'Anonymous Message'
                    current_discriminator = value >> 10 & 0x3FFF
                    message_name = get_message_name(current_message_data)
                    
                
                else: 
                    current_message_data = None
                    current_message_type = 'Unknown Message Type'
                self.name = message_name
                    

        if frame.type == 'control_field':
            #num = frame.data['num_data_bytes']
            #self.data.append(num)
            self.numb_msgs = frame.data['num_data_bytes']

        if frame.type == 'data_field':
            num = frame.data['data']
            self.data.append(int.from_bytes(num, "little"))
            self.number_of_data_frames = self.number_of_data_frames + 1
            
            if(self.number_of_data_frames == 8):
                if(int.from_bytes(num,"big") & 0b10000000):
                    self.message_start = self.frame_start
                    self.started = True
                if(int.from_bytes(num,"big") & 0b1000000):
                    self.last_message = True
            
            elif(self.numb_msgs == self.number_of_data_frames):
                if(int.from_bytes(num,"big") & 0b10000000):
                    self.message_start = self.frame_start
                    self.started = True
                self.last_message = True
            
        if frame.type == 'crc_field':
            # Do Nothing for now, maybe add CRC checking later
            fn = 2

        if frame.type == 'ack_field':

            self.frames.append(dronecan.transport.Frame(
                    message_id = self.id,
                    data = self.data,
                    ts_real = 0,
                    canfd = False
                ))
            
            if(self.last_message and self.started):
                T = dronecan.transport.Transfer()
                
                try: 
                    T.from_frames(self.frames)              # Combines individual frames into a single transfer
                    value = dronecan.to_yaml(T.payload)     # Decodes message payload into YAML
                        
                except Exception:
                    value = "Exception occured :("
                
                self.multi_message = False
                self.started = False
                
                return AnalyzerFrame('Full-Frame',self.message_start, frame.end_time,{
                    'Name' : self.name,
                    'Data' : value
                })
    
    def decode_mavlink(self, frame: AnalyzerFrame):
        if frame.type == 'data_field':
            raw = frame.data['data']
            self.mavlink_buf += raw  # make sure self.mavlink_buf = b'' in __init__

            # Track start time of the message
            if not self.mavlink_started:
                self.message_start = frame.start_time
                self.mavlink_started = True

        if frame.type == 'ack_field':
            if not self.mavlink_buf:
                return

            try:
                # pymavlink needs a file-like object -> wrap a dummy IO object
                f = io.BytesIO()
                
                if self.protocol_mode == 'MAVLink2':
                    mav = mavlink2.MAVLink(f)
                else:
                    mav = mavlink1.MAVLink(f)

                # Parse byte by byte — parse_char accumulates internally
                # and returns a message only when a complete one is found
                result = None
                for byte in self.mavlink_buf:
                    msg = mav.parse_char(bytes([byte]))
                    if msg is not None:
                        result = msg

                self.mavlink_buf = b''
                self.mavlink_started = False

                if result is not None:
                    return AnalyzerFrame('Full-Frame', self.message_start, frame.end_time, {
                        'Name': result.get_type(),
                        'Data': str(result.to_dict())
                    })

            except Exception as e:
                self.mavlink_buf = b''
                self.mavlink_started = False
                return AnalyzerFrame('Full-Frame', self.message_start, frame.end_time, {
                    'Name': 'Error',
                    'Data': f'Exception: {e}'
                })
        
    def decode(self, frame: AnalyzerFrame):
        if self.protocol_mode == 'DroneCAN':
            return self.decode_dronecan(frame)
        elif self.protocol_mode == 'MAVLink1' or self.protocol_mode == 'MAVLink2':
            return self.decode_mavlink(frame)
        

        
