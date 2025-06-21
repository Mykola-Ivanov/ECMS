import subprocess

# Define the command you want to execute
def run_cppcheck(command, output_file):
    try:
        # Execute the command and capture its output
        result = subprocess.run(command, capture_output=True, text=True, check=True)

        # Open a file in write mode
        with open(output_file, "w") as f:
            # Write the captured standard output to the file
            f.write("--- Standard Output ---\n")
            f.write(result.stdout)
            
            # If there's any standard error, write it as well
            if result.stderr:
                f.write("\n--- Standard Error ---\n")
                f.write(result.stderr)

        print(f"Cppcheck output successfully saved to {output_file}.")

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

    command_string = 'C:/Users/mykol/.platformio/penv/Scripts/platformio.exe check --environment upesy_wroom'
    output_file = 'analysis.txt'
    run_cppcheck(command_string , output_file)

