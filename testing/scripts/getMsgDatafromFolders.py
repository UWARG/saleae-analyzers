"""
Get the DroneCAN message data from nests of folders (dsdl_specs) and save it to a .csv file.
"""

import os
import re
import json

def build_dsdl_lookup(root_dir='dsdl_specs', output_file='dsdl_lookup.json'):
    # Dictionary to store number -> message name mappings
    lookup_dict = {}

    processed_count = 0
    skipped_count = 0
    
    # Pattern to match files starting with numbers
    number_pattern = re.compile(r'^(\d+)\..+')
    
    # Go through all the directories
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            # Check if filename starts with a number
            match = number_pattern.match(filename)
            if match:
                number = match.group(1)
                parts = filename.split('.')
                
                # Get the message name (part between first and second dot)
                if len(parts) >= 2:
                    message_name = parts[1]
                    lookup_dict[number] = message_name
                    processed_count += 1
                else:
                    skipped_count += 1
    
    # Save to JSON file
    with open(output_file, 'w') as f:
        json.dump(lookup_dict, f, indent=2)
    
    print(f"Done! Processed {processed_count} files, skipped {skipped_count}")
    
    return lookup_dict

build_dsdl_lookup()



    