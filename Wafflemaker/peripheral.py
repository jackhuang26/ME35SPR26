import pigpio
import time

I2C_ADDR = 0x08
pi = pigpio.pi()

def run_slave():
    # Initialize the BSC for Pi 4 (GPIO 10=SDA, 11=SCL)
    # This automatically uses the correct pins for Pi 4
    pi.bsc_i2c(I2C_ADDR)
    print(f"Pi 2 (Slave) active on 0x{I2C_ADDR:02x}. Waiting for Pi 1...")

    try:
        # 1. Wait for "task 1 complete"
        while True:
            status, bytes_read, data = pi.bsc_i2c(I2C_ADDR)
            if bytes_read > 0:
                msg = data.decode('ascii', 'ignore').strip()
                if "task 1 complete" in msg:
                    print(f"Pi 2: Received '{msg}'")
                    break
            time.sleep(0.05)

        # 2. Simulate Task 2
        print("Pi 2: Starting Task 2...")
        time.sleep(2) 

        # 3. Send "task 2 complete"
        response = "task 2 complete"
        pi.bsc_i2c(I2C_ADDR, response.encode())
        print(f"Pi 2: Task 2 finished. Response loaded into buffer.")

        # 4. Stay alive until the Master (Pi 1) reads the data
        while True:
            status, b_read, d = pi.bsc_i2c(I2C_ADDR)
            # Check if TX FIFO is empty (bits 16-21 of status)
            tx_fifo = (status >> 16) & 0x3F
            if tx_fifo == 0:
                print("Pi 2: Master has read the message. Task cycle complete.")
                break
            time.sleep(0.1)

    finally:
        pi.bsc_i2c(0) # Disable I2C slave
        pi.stop()

run_slave()