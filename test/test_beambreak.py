import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from manipulator import Claw

claw = Claw()

print("Insert object to break beam")

while True:

    if claw.is_object_detected():
        print("Object detected!")

    time.sleep(0.1)