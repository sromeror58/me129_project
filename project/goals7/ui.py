import threading
import time

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = 0
        self.wall_following_side = "left"

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        return self.lock.release()

def runui(shared):
    """
    UI thread function that handles user input and updates the shared mode.
    Args:
        shared: SharedState object containing the mode and lock
    """
    print("\nWall Following Control UI")
    print("Available commands:")
    print("  'discrete'   - Follow wall using discrete approach")
    print("  'continuous' - Follow wall using continuous approach")
    print("  'stop'       - Stop the robot")
    print("  'quit'       - Exit the program")
    
    while True:
        try:
            # Grab the user input - implicitly sleep while waiting
            command = input("\nEnter command: ").strip().lower()
            
            # Grab access to the shared data
            if shared.acquire():

                # Immediately process the command into the shared flags
                if command == 'discrete':
                    shared.mode = 1
                    print("Switching to discrete wall following mode")
                    side = input("Choose wall side to follow (l/r): ").strip().lower()                    
                    shared.wall_following_side = side

                elif command == 'continuous':
                    shared.mode = 2
                    print("Switching to continuous wall following mode")
                    side = input("Choose wall side to follow (l/r): ").strip().lower()                    
                    shared.wall_following_side = side

                elif command == 'stop':
                    shared.mode = 0
                    print("Stopping robot")

                elif command == 'quit':
                    shared.mode = -1
                    shared.release()
                    print("Exiting program...")
                    break

                else:
                    print("Unknown command. Please use: discrete, continuous, stop, or quit")
                
                # Release the shared data
                shared.release()
            
        except KeyboardInterrupt:
            if shared.acquire():
                shared.mode = -1
                shared.release()
            print("\nExiting due to keyboard interrupt...")
            break

        except Exception as e:
            print(f"Error in UI thread: {e}")
            continue
