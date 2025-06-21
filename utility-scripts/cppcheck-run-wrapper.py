# Run cppcheck and save the output to a file
# 
# Usage: python cppcheck_run_wrapper.py

import subprocess

# Define the command you want to execute
def run_cppcheck(command, output_file):
    try:
        # Execute the command and capture its output
        result = subprocess.run(f"{command} 2> {output_file}", check=True)

        # Check if the command was successful
        if result.returncode == 0:
            print("Cppcheck executed successfully.")
        else:
            print(f"Cppcheck failed with return code: {result.returncode}")

    except subprocess.CalledProcessError as e:
        # Handle cases where the command returns a non-zero exit code (error)
        print(f"Error executing command: {e}")
        print(f"Stderr: {e.stderr}")
    except FileNotFoundError:
        print(f"Error: Command '{command[0]}' not found. Make sure it's in your system's PATH.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    import sys

    if len(sys.argv) == 2:
        print("Usage: python cppcheck_run_wrapper.py")
        sys.exit(1)

    command_string = 'cppcheck src/ --xml --xml-version=2 --enable=all --addon=cppcheck-config/misra.json --language=c --std=c11'
    output_file = 'report_cppcheck.xml'
    run_cppcheck(command_string , output_file)

