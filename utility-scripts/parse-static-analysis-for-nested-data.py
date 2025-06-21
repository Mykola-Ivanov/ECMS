
# This script parses analysis.txt results to extract it for Nested Data Reporting Jenkins Plugin.
# 
# Usage: python parse-static-analysis-for-nested-data.py <input_file> <output_file>
import sys
import json
import re
import os

def parse_line(line):
    # Parse a single line of the static analysis output.
    # Example line: "src\main.c:23: [low:style] misra violation (rule-texts-file not found: misra.txt) [misra-c2012-2.5]"
    # Expected rusult: 
    # {
        # "file": "src/main.c",
        # "line": 23,
        # "severity": "low",
        # "type": "style",
        # "message": "rule-texts-file not found: misra.txt",
        # "violation": "misra-c2012-2.5"
    # }

    # Replace \ with / in file paths for consistency
    line = line.replace('\\', '/')

    # Explanation of the regex:
    # ^(.*?):(\d+): - Matches the file path and line number
    # \s*\[(\w+):(\w+)\] - Matches the severity and type
    # \s*(.*?) - Matches the message
    # (?:\s*\[(.*?)\])? - Optionally matches the violation at the end
    # The regex captures:
    # 1. File path
    # 2. Line number
    # 3. Severity
    # 4. Type
    # 5. Message
    # 6. Violation (optional)
    match = re.match(
        r'^(.*?):(\d+):\s*\[(\w+):(\w+)\]\s*(.*?)(?:\s*\[(.*?)\])?$',
        line
    )

    if match:
        return {
            'file': match.group(1).replace('\\', '/'),  # Normalize file path
            'line': int(match.group(2)),
            'severity': match.group(3),
            'type': match.group(4),
            'message': match.group(5),
            'violation': match.group(6)
        }
    return None

def select_line_of_interest(lines, begin_marker, end_marker):
    # Select lines of interest between begin_marker and end_marker
    # This function is not used in the current implementation but can be useful for future enhancements
    selected_lines = []
    # regexp_begin = None
    # regexp_end = None
    selection_begin = False
    selection_end = False

    # if begin_marker is not None:
    # begin_marker = re.compile(r'\s*' + re.escape(begin_marker) + r'\s*')
    # if end_marker is not None:
    # end_marker = re.compile(r'\s*' + re.escape(end_marker) + r'\s*')

    for line in lines:
        if (begin_marker == None) or (begin_marker.match(line)):
            selection_begin = True
            print(f"Selection begin at line: {line.strip()}")  # Debugging output
        if ((end_marker is not None) and end_marker.match(line)):
            print(f"Selection end at line: {line.strip()}")  # Debugging output
            selection_end = True
        if selection_begin and selection_end:
            print(f"Selection complete. Breaking out of loop.")  # Debugging output
            break
        if selection_begin and not selection_end:
            # If we are in the selection phase, add the line to the selected lines
            selected_lines.append(line.strip())

    return selected_lines

def parse_static_analysis(input_file, output_file, begin_marker, end_marker):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    lines = select_line_of_interest(lines, begin_marker, end_marker)  # This function is not used in the current implementation but can be useful for future enhancements
    results = []
    current_file = None                                           # Current file being processed
    current_line = None                                           # Current line number being processed
    current_message = None                                        # Current message being processed
    # Iterate through each line in the input file

    for line in lines:
        line = line.strip()
        if not line:
            continue

        # Match the file path
        parse_line_result = parse_line(line)
        if parse_line_result:
            # If we have a previous result, save it before starting a new one
            if current_file is not None and current_line is not None and current_message is not None:
                results.append({
                    'file': current_file,
                    'line': current_line,
                    'message': current_message
                })

            # Start a new result
            current_file = parse_line_result['file']
            current_line = parse_line_result['line']
            current_message = parse_line_result['message']
        else:
            # If the line does not match the expected format, append it to the current message
            if current_message is not None:
                current_message += ' ' + line

    # Add the last result if it exists
    if current_file is not None and current_line is not None and current_message is not None:
        results.append({
            'file': current_file,
            'line': current_line,
            'message': current_message
        })

    with open(output_file, 'w') as outfile:
        json.dump(results, outfile, indent=4)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python parse_static_analysis_for_nested_data.py <input_file> <output_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Input file '{input_file}' does not exist.")
        sys.exit(1)
    begin_marker = re.compile(r'\s*--------------------------------\s*')                              # Default begin marker
    end_marker   = re.compile(r'\s*========================= \[PASSED\] Took \d+\.\d+ seconds =========================\s*')  # Default end marker

    parse_static_analysis(input_file, output_file, begin_marker, end_marker)
    print(f"Parsed results saved to '{output_file}'.")
    sys.exit(0)