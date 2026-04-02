# UWARG High Level Analyzer
from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting
from pathlib import Path

import re
import sys
import os
import json

from pathlib import Path

lib_path = str(Path(__file__).parent / "lib")
if lib_path not in sys.path:
    sys.path.append(lib_path)

import dronecan

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

def format_payload(raw_yaml: str) -> str:
        if not raw_yaml:
            return ''

        normalised = raw_yaml.replace('\r\n', ' ').replace('\r', ' ').replace('\n', ' ')
        normalised = ' '.join(normalised.split())   # collapse any double-spaces

        piped = re.sub(r'(\S)\s+(\w+:)', r'\1 | \2', normalised)

        def trim_float(m):
            return format(float(m.group()), '.2f').rstrip('0').rstrip('.') or '0'

        piped = re.sub(r'\b\d+\.\d+\b', trim_float, piped)

        return piped

class Hla(HighLevelAnalyzer):
    result_types = {
        # DroneCAN frame types — bubble shows: name | src | data
        'Standard Message': {
            'format': '{{data.Name}} | {{data.Source}} | {{data.Data}}'
        },
        'Request Message': {
            'format': '{{data.Name}} | {{data.Source}} → {{data.Dest}} | {{data.Data}}'
        },
        'Response Message': {
            'format': '{{data.Name}} | {{data.Source}} → {{data.Dest}} | {{data.Data}}'
        },
        'Anonymous Message': {
            'format': '{{data.Name}} | {{data.Data}}'
        },
        'Decode Error': {
            'format': 'Error | {{data.Name}} | {{data.Data}}'
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
        self.message_type = None
        self.source_node_id = None
        self.dest_node_id = None
        self.data = []
        self.frames = []
        self.id = None

    def decode_dronecan(self, frame: AnalyzerFrame):
        if frame.type == 'identifier_field':
            
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
                current_service = value >> 7 & 0x01       
                current_source_node_id = value & 0x7F      
                current_priority = value >> 24 & 0x1F  
                
                if not current_service and current_source_node_id:
                    current_message_data = value >> 8 & 0xFFFF
                    message_name = get_message_name(current_message_data)
                    current_message_type = 'Standard Message'
                    current_destination_node_id = None
 
                elif current_service and current_source_node_id:
                    current_message_data = value >> 16 & 0xFF
                    current_destination_node_id = value >> 8 & 0x7F
                    message_name = get_message_name(current_message_data)
                    if value >> 15 & 0x01:
                        current_message_type = 'Request Message'
                    else:
                        current_message_type = 'Response Message'
 
                elif not current_service and not current_source_node_id:
                    current_message_data = value >> 8 & 0x3
                    current_message_type = 'Anonymous Message'
                    current_discriminator = value >> 10 & 0x3FFF
                    message_name = get_message_name(current_message_data)
                    current_destination_node_id = None
                    
                else: 
                    current_message_data = None
                    current_message_type = 'Unknown Message Type'
                    current_destination_node_id = None
 
                self.name = message_name
                self.message_type   = current_message_type
                self.source_node_id = current_source_node_id
                self.dest_node_id   = current_destination_node_id
 
        if frame.type == 'control_field':
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
            self.frame_crc = frame.data.get('crc', None)
 
        if frame.type == 'ack_field':
            self.frames.append(dronecan.transport.Frame(
                    message_id = self.id,
                    data = self.data,
                    ts_real = 0,
                    canfd = False
                ))
            
            if self.last_message and self.started:
                T = dronecan.transport.Transfer()

                try:
                    T.from_frames(self.frames)                      # also validates CRC and other invariants
                    raw_yaml = dronecan.to_yaml(T.payload)
                    value = format_payload(raw_yaml)
                    had_error = False

                except Exception as e:
                    print("DECODE ERROR:", e)
                    value = str(e)
                    had_error = True

                frame_type = 'Decode Error' if had_error else self.message_type

                frame_data = {
                    'Name'   : self.name,
                    'Type'   : self.message_type,
                    'Source' : f'Node {self.source_node_id}',
                    'Data'   : value,
                }

                if self.dest_node_id is not None:
                    frame_data['Dest'] = f'Node {self.dest_node_id}'

                return AnalyzerFrame(frame_type, self.message_start, frame.end_time, frame_data)
 
    def decode(self, frame: AnalyzerFrame):
        return self.decode_dronecan(frame)
