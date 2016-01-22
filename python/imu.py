# -*- coding: utf-8 -*-
"""
      Microstrain 3DM-GX3-35 Interaction Module
      =========================================
      
   Interact with microstrain GX3-35 IMU to get accelerometer, gyro, 
   magnetometer and GPS data. The imu also outputs Euler angles, Quaternion
   and orientation matrix in addition to many other values. 

"""

import sys, serial, struct
from time import sleep
from StringIO import StringIO
from optparse import OptionParser

from mpi import MPI


PI = 3.141592653589793
g = 9.80665                # acceleration due to gravity in m/sec^2
CMDS = ['ping', 'idle', 'resume', 'info', 'test', 'reset', 'euler', 'quaternion', 'accelerometer', 'gyro', 'magneto', 'uav']



def rad2deg(angle):
    """
    Convert radian to degree.
    
    Parameters
    ----------
    angle : float
        Angle in radians
        
    Returns
    -------
    degree : float
        Angle in degrees
    """
    return (180./PI) * angle


def deg2rad(angle):
    """
    Convert degree to radian.
    
    Parameters
    ----------
    angle : float
        Angle in degrees.
        
    Returns
    -------
    radians : float
        Angle in radians.
    """
    return (PI/180.) * angle


class IMU(object):
    def __init__(self, port = '/dev/ttyACM0', baudrate = 115200, timeout = 3.0):
        """
        IMU class constructor.
        
        Parameters
        ----------
        port : string
            Serial port where IMU is connected. Default on linux is /dev/ttyACM0
            
        baudrate : int
            Serial baudrate. Default for microstrain imu is 115200 bytes/sec.
            
        timeout : float
            Serial timout in seconds. Default is 3 seconds.
            
        Returns
        -------
        None
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # Setup serial port
        self.ser = serial.Serial(self.port, self.baudrate, timeout = self.timeout)

    
    def ping(self):
        """
        Send a 'ping' command to the imu.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Ping payload
        field_desc = 0x01
        field_len = 0x02
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(10)
        
        if reply[7] == "\x00":
            print "  Ping successful!"
        else:
            print "  Ping unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        

    def set_to_idle(self):
        """
        Place device into idle mode. Device responds with ACK if successfully 
        placed in idle mode. This command will suspend streaming (if enabled) 
        or wake the device from sleep (if sleeping) to allow it to respond to 
        status and setup commands. You may restore the device mode by issuing 
        the Resume command.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_len = 0x02
        field_desc = 0x02
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(10)
        
        if reply[7] == "\x00":
            print " IMU placed in idle mode."
        else:
            print "  Command unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
    
    
    def resume(self):
        """
        Place device back into the mode it was in before issuing the Set To 
        Idle command. If the Set To Idle command was not issued, then the 
        device is placed in default mode.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_len = 0x02
        field_desc = 0x06
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(10)
        
        if reply[7] == "\x00":
            print "  Resume successful!"
        else:
            print "  Command unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return

        
    def get_device_info(self):
        """
        Get the device ID strings and firmware version. Reply has two fields: 
        “ACK/NACK” and “Device Info Field”.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_len = 0x02
        field_desc = 0x03
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(94)
        
        if reply[7] == "\x00":
            print "  IMU information received!"
            print "    Firmware version : %d" %int(reply[10:12].encode('hex'), 16) 
            print "    Model Name       : %s" %reply[12:28].encode('ascii')
            print "    Model Number     : %s" %reply[28:44].encode('ascii')
            print "    Serial Number    : %s" %reply[44:60].encode('ascii')
            print "    Lot Number       : %s" %reply[60:76].encode('ascii')
            print "    Device Options   : %s" %reply[76:92].encode('ascii')
        else:
            print "  Command unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        
        
    def device_test(self):
        """
        Run the device Built-In Test (BIT). The Built-In Test command always 
        returns a 32 bit value. A value of 0 means that all tests passed. A 
        non-zero value indicates that not all tests passed. The failure flags 
        are device dependent. The flags for the 3DM-GX3-35 are defined below.
        
        The BIT will take approximately 5 seconds to complete on the 3DM-GX3-35. 
        The GPS power will be cycled during the test resulting in the temporary 
        loss and subsequent recalculation of the position solution.
        
        Byte     Byte 1(LSB)     Byte 2     Byte 3     Byte 4(MSB)
        Device  AP-1 Processor    AHRS       GPS         Reserved
        =========================================================== 
        Bit 1 (LSB)I2C Hardware Error Communication Error Communication  Error Reserved
        Bit 2 I2C EEPROM Error Reserved 1PPS Signal Error Reserved
        Bit 3 Reserved Reserved 1 PPS Inhibit Error Reserved
        Bit 4 Reserved Reserved Power Control Error Reserved
        Bit 5 Reserved Reserved Reserved Reserved
        Bit 6 Reserved Reserved Reserved Reserved
        Bit 7 Reserved Reserved Reserved Reserved
        Bit 8 (MSB) Reserved Reserved Reserved Reserved
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_len = 0x02
        field_desc = 0x05
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(16)
        
        if reply[7] == "\x00":
            print "  Device Built-In Test (BIT) successful!"
            print "  BIT Error Flags : "
            print "    Byte 1 : ", '0x' + reply[10].encode('hex') 
            print "    Byte 2 : ", '0x' + reply[11].encode('hex')
            print "    Byte 3 : ", '0x' + reply[12].encode('hex')
            print "    Byte 4 : ", '0x' + reply[13].encode('hex')
        else:
            print "  Command unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        
        
    def reset(self):
        """
        Resets the 3DM-GX3-35. This command has a single 32 bit security value 
        parameter. Device responds with ACK if it recognizes the command and 
        then immediately resets.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_BASE_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_len = 0x02
        field_desc = 0x7E
        mpi_packet.payload = [field_len, field_desc]

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(10)
        
        if reply[7] == "\x00":
            print "  IMU reset successful!"
        else:
            print "  IMU reset unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        
        
    def get_euler_angles(self):
        """
        Get Pitch, Roll and Yaw angles.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x01, 0x0C, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(30)
        
        if reply[7] == "\x00":
            roll = rad2deg(struct.unpack('>f', reply[16:20])[0])
            pitch = rad2deg(struct.unpack('>f', reply[20:24])[0])
            yaw = rad2deg(struct.unpack('>f', reply[24:28])[0])
        
            print "  Got Euler angles successfully!"
            print "  Euler Angles (in degrees) : "
            print "    Roll : %6.4f" %roll
            print "    Pitch : %6.4f"  %pitch
            print "    Yaw : %6.4f"  %yaw
        else:
            print "  IMU Euler angles unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return (roll, pitch, yaw)
        
        
    def get_quaternion(self):
        """
        This method returns a 4 component quaternion which describes the 
        orientation of the 3DM-GX3 with respect to the fixed earth coordinate 
        quaternion.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x01, 0x0A, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(34)
        
        if reply[7] == "\x00":
            print "  Got Quaternion successfully!"
            print "  Quaternions : "
            print "    q0 : %6.6f" %struct.unpack('>f', reply[16:20])[0]
            print "    q1 : %6.6f"  %struct.unpack('>f', reply[20:24])[0]
            print "    q2 : %6.6f"  %struct.unpack('>f', reply[24:28])[0]
            print "    q3 : %6.6f"  %struct.unpack('>f', reply[28:32])[0]
        else:
            print "  IMU Quaternion unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        
    def get_scaled_accelerometer(self):
        """
        This is a vector quantifying the direction and magnitude of the 
        acceleration that the 3DMGX3 s exposed to. This quantity is derived 
        from Raw Accelerometer, but is fully temperature compensated and scaled 
        into physical units of g (1 g = 9.80665 m/sec^2). It is expressed in 
        terms of the 3DM-GX3®’s local coordinate system.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x01, 0x04, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(30)
        
        if reply[7] == "\x00":
            print "  Got Scaled Accelerometer values successfully!"
            print "  Scaled Accelerometer (in m/sec^2) : "
            print "    X : %6.4f" %(struct.unpack('>f', reply[16:20])[0] * 9.080665)
            print "    Y : %6.4f" %(struct.unpack('>f', reply[20:24])[0] * 9.80665)
            print "    Z : %6.4f"  %(struct.unpack('>f', reply[24:28])[0] * 9.80665)
        else:
            print "  IMU Scaled Accelerometer unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return
        
    def get_scaled_gyro(self):
        """
        This is a vector quantifying the rate of rotation (angular rate) of the 
        3DM-GX3®. This quantity is derived from the Raw Angular Rate quantities, 
        but is fully temperature compensated and scaled into units of 
        radians/second. It is expressed in terms of the 3DM-GX3®’s local 
        coordinate system in units of radians/second.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x01, 0x05, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(30)
        
        if reply[7] == "\x00":
            gyro_x = rad2deg(struct.unpack('>f', reply[16:20])[0])
            gyro_y = rad2deg(struct.unpack('>f', reply[20:24])[0])
            gyro_z = rad2deg(struct.unpack('>f', reply[24:28])[0])
            
            print "  Got Scaled Gyro values successfully!"
            print "  Scaled Gyro Vector (in degrees/sec) : "
            print "    X : %6.4f" %gyro_x
            print "    Y : %6.4f" %gyro_y
            print "    Z : %6.4f" %gyro_z
        else:
            print "  IMU Scaled Gyro unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return (gyro_x, gyro_y, gyro_z)
        
                                                                                                                                                                                                                                                
    def get_scaled_magneto(self):
        """
        This is a vector which gives the instantaneous magnetometer direction 
        and magnitude. It is fully temperature compensated and is expressed in 
        terms of the 3DM-GX3®’s local coordinate system in units of Gauss.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x01, 0x06, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.002)
        reply = self.ser.read(30)
        
        if reply[7] == "\x00":
            magneto_x = struct.unpack('>f', reply[16:20])[0]
            magneto_y = struct.unpack('>f', reply[20:24])[0]
            magneto_z = struct.unpack('>f', reply[24:28])[0]
            
            print "  Got Scaled Magnetometer values successfully!"
            print "  Scaled Magnetometer values (in Gauss) : "
            print "    X : %6.4f" %magneto_x
            print "    Y : %6.4f" %magneto_y
            print "    Z : %6.4f" %magneto_z
        else:
            print "  IMU Scaled Magnetometer unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return (magneto_x, magneto_y, magneto_z)
        
        
    def get_uav(self):
        """
        Get Pitch, Roll and Yaw angles along with gyro and magnetometer values.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        # Create a MPI packet object
        mpi_packet = MPI()
        mpi_packet.descriptor = MPI.MPI_3DM_CMD_DESCRIPTOR
        
        # Set to idle payload
        field_desc = 0x01
        field_data = 0x00, 0x04, 0x0C, 0x00, 0x00, 0x04, 0x00, 0x00, 0x05, 0x00, 0x00, 0x06, 0x00, 0x00
        field_len = len(field_data) + 2
        
        mpi_packet.payload = [field_len, field_desc]
        for value in field_data:
            mpi_packet.payload.append(value)

        # Payload length        
        mpi_packet.payload_len = len(mpi_packet.payload)
        
        # Build imu ping command in bytes
        command = mpi_packet.build()
                
        # Send byte packet to microstrain imu       
        self.ser.write(command)
        
        # Read output from the imu adter sleeping for 2 ms
        sleep(0.001)
        reply = self.ser.read(72)
        
        if reply[7] == "\x00":
            print "  IMU UAV Successful!"
            
            roll = rad2deg(struct.unpack('>f', reply[16:20])[0])
            pitch = rad2deg(struct.unpack('>f', reply[20:24])[0])
            yaw = rad2deg(struct.unpack('>f', reply[24:28])[0])
        
            accl_x = struct.unpack('>f', reply[30:34])[0] * g
            accl_y = struct.unpack('>f', reply[34:38])[0] * g
            accl_z = struct.unpack('>f', reply[38:42])[0] * g
        
            gyro_x = rad2deg(struct.unpack('>f', reply[44:48])[0])
            gyro_y = rad2deg(struct.unpack('>f', reply[48:52])[0])
            gyro_z = rad2deg(struct.unpack('>f', reply[52:56])[0])
            
            magneto_x = struct.unpack('>f', reply[58:62])[0]
            magneto_y = struct.unpack('>f', reply[62:66])[0]
            magneto_z = struct.unpack('>f', reply[66:70])[0]
        else:
            print "  IMU UAV unsuccessful"
            err = '0x' + reply[7].encode('hex')
            print "  Error Code : ", err
            print "  Error Message : ", MPI.MPI_ACK_NACK_ERROR[err]
            
        return (roll, pitch, yaw, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z, magneto_x, magneto_y, magneto_z)




def process(cmd, port, baudrate, timeout):
    """
    
    """
    imu = IMU(port, baudrate, timeout)
    
    if cmd == 'ping':
        imu.ping()
    elif cmd == 'idle':
        imu.set_to_idle()
    elif cmd == 'resume':
        imu.resume()
    elif cmd == 'info':
        imu.get_device_info()
    elif cmd == 'test':
        imu.device_test()
    elif cmd == 'reset':
        imu.reset()
    elif cmd == 'euler':
        imu.get_euler_angles()
    elif cmd == 'quaternion':
        imu.get_quaternion()
    elif cmd == 'accelerometer':
        imu.get_scaled_accelerometer()
    elif cmd == 'gyro':
        imu.get_scaled_gyro()
    elif cmd == 'magneto':
        imu.get_scaled_magneto()
    elif cmd == 'uav':
        imu.get_uav()
    

if __name__ == "__main__":
    usage = "Usage: python %prog [options] command"
    description = "Description. Utility to interact with Microstrain 3DM-GX3-35 IMU."
    parser = OptionParser(usage = usage, version = "%prog 0.1dev", description = description)
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default = False,
                      help = "print result messages to stdout"
                      )
    parser.add_option("-q", "--quiet",
                    action="store_false", dest="verbose", default = True,
                    help = "don't print result messages to stdout"
                    )
    parser.add_option("-p", "--port", dest = "port",
                    action='store', metavar="PORT", help = "Microstrain IMU port (default is /dev/ttyACM0)",
                    default = '/dev/ttyACM0'
                    )
    parser.add_option("-b", "--baudrate", dest = "baudrate",
                    action='store', metavar="BAUDRATE", help = "Serial port baudrate (default is 115200)",
                    default = 115200
                    )                    
    parser.add_option("-t", "--timeout", dest = "timeout",
                    action='store', metavar="TIMEOUT", help = "Serial connection timeout in seconds (default is 3 seconds)",
                    default = 3.0
                    )     
                                        
    (options, args) = parser.parse_args()  
    if len(args) != 1:
        parser.error("Incorrect number of arguments")
    else:
        if args[0] not in CMDS:
            print "Only IMU commands ", CMDS, " are supported"
            sys.exit()

    print "IMU: Utility to interact with Microstrain 3DM-GX3-35 IMU"
    print "  IMU command : ", args[0]
        
    # Check verbosity
    if not options.verbose:
        output = StringIO()
        old_stdout = sys.stdout
        sys.stdout = output
 
    process(args[0], options.port, options.baudrate, options.timeout)    

    # Reset verbosity
    if not options.verbose:
        sys.stdout = old_stdout   
