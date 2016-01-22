
class MPI:
    MPI_BASE_CMD_DESCRIPTOR = 0x01
    MPI_3DM_CMD_DESCRIPTOR  = 0x0C
    
    MPI_ACK_NACK_ERROR = {'0x00': 'MIP_ACK_NACK_ERROR_NONE', '0x01': 'MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND', '0x02': 'MIP_ACK_NACK_ERROR_CHECKSUM_INVALID', '0x03': 'MIP_ACK_NACK_ERROR_PARAMETER_INVALID', '0x04': 'MIP_ACK_NACK_ERROR_COMMAND_FAILED', '0x05': 'MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT'}

        
    def __init__(self):
        """
        MPI packet.
        """
        self.sync1 = 0x75
        self.sync2 = 0x65
        self.descriptor = 0
        self.payload_len = 0
        self.payload = 0
        self.checksum_msb = 0
        self.checksum_lsb = 0
        

    def checksum(self, bytestr):
        """
        """
        ba = bytearray(bytestr)
        
        nele = len(ba)
        chksum1 = 0
        chksum2 = 0
        
        for i in range(nele):
            chksum1 += ba[i]
            chksum2 += chksum1
        
        return (chksum1 & 0xff, chksum2 & 0xff)

                
    def build(self):
        # Determine command checksum
        mpi_byte_str = chr(self.sync1) + chr(self.sync2) + chr(self.descriptor) + chr(self.payload_len)
        for i in range(self.payload_len):
             mpi_byte_str += chr(self.payload[i])
        
        self.checksum_msb, self.checksum_lsb = self.checksum(mpi_byte_str)
        
        mpi_byte_str += chr(self.checksum_msb) + chr(self.checksum_lsb)
        
        return mpi_byte_str
        
    
    def show(self):
        print "SYNC1        : %5s" %hex(self.sync1)
        print "SYNC2        : %5s" %hex(self.sync2)
        print "DESCRIPTOR   : %5s" %hex(self.descriptor)
        print "PAYLOAD LEN  : %5s" %hex(self.payload_len)
        print "PAYLOAD      : "
        for i in range(self.payload_len):
            print "               %5s" %hex(self.payload[i])
        print "CHECKSUM MSB : %5s" %hex(self.checksum_msb)
        print "CHECKSUM LSB : %5s" %hex(self.checksum_lsb)