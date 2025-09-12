import time
import math

from .bytes import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

class sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, STS_ACC, 7)
    
    # ---- Backwards-compatible aliases ----
    ReadByte  = protocol_packet_handler.read1ByteTxRx
    Read2Byte = protocol_packet_handler.read2ByteTxRx
    Read4Byte = protocol_packet_handler.read4ByteTxRx
    WriteByte  = protocol_packet_handler.write1ByteTxRx
    Write2Byte = protocol_packet_handler.write2ByteTxRx
    Write4Byte = protocol_packet_handler.write4ByteTxRx
    
    # --- High - level helpers

    def ChangeID(self, sts_id, new_id):
        try:
            new_id = int(new_id)
        except (TypeError, ValueError):
            return f"Invalid new_id {new_id}: must be an integer"
        
        if not (0 <= new_id <= 253):
            return f"new_id {new_id} must be between 0 and 253"


        sts_model_number, sts_comm_result, sts_error = self.ping(sts_id)
        if sts_comm_result != COMM_SUCCESS:
            return f"Could not ping servo: {sts_id}: {self.getTxRxResult(sts_comm_result)}"
        if sts_error != 0:
            return f"Servo {sts_id} returned error: {self.getRxPacketError(sts_error)}"

        sts_comm_result, sts_error = self.unLockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to unLockEprom: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        
        sts_comm_result, sts_error = self.write1ByteTxRx(sts_id, STS_ID, new_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            # Try to re-lock under old ID
            try:
                self.LockEprom(sts_id)
            finally:
                pass
            return f"Failed to change ID: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        
        # allow for settle
        time.sleep(0.05)

         # Verify by pinging the new ID
        _, sts_comm_result, sts_error = self.ping(new_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            # Try to lock with both IDs just in case
            try:
                self.LockEprom(new_id)
            except Exception:
                try:
                    self.LockEprom(sts_id)
                except Exception:
                    pass
            return f"Write reported success, but ping to new ID {new_id} failed: sts_comm_result={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"

        # Lock EPROM under the new ID
        sts_comm_result, sts_error = self.LockEprom(new_id)
        # If locking fails we still changed the ID; report but consider operation successful
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"ID changed to {new_id}, but LockEprom failed: sts_comm_result={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"

        return None




        

    def ChangeMode(self, sts_id, mode):
        # Unlock
        sts_comm_result, sts_error = self.unLockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to unLockEprom: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        
        # Write mode
        sts_comm_result, sts_error = self.write1ByteTxRx(sts_id, STS_MODE, mode & 0xFF)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to write mode: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        
        # allow for settle
        time.sleep(0.05)

        # Verify
        val, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, STS_MODE)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to verify mode: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        if val != mode:
            return f"Verify mismatch: wrote {mode}, read back {val}"
        
        # Re-lock
        sts_comm_result, sts_error = self.LockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Mode set to {mode}, but LockEprom failed: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"

        return None
    
    def ChangeMaxLimit(self, sts_id, limit):
        # Unlock
        sts_comm_result, sts_error = self.unLockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to unLockEprom: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        # Change Limit
        sts_comm_result, sts_error = self.write2ByteTxRx(sts_id, STS_MAX_ANGLE_LIMIT_L, limit & 0xFFFF)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to write limit: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        
        # Re-lock
        sts_comm_result, sts_error = self.LockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Limit cahchanged to {limit & 0xFFFF}, but LockEprom failed: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"

        return None

    def ChangeMinLimit(self, sts_id, limit):
        # Unlock
        sts_comm_result, sts_error = self.unLockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to unLockEprom: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        # Change Limit
        sts_comm_result, sts_error = self.write2ByteTxRx(sts_id, STS_MIN_ANGLE_LIMIT_L, limit & 0xFFFF)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Failed to write limit: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"

        # Re-lock
        sts_comm_result, sts_error = self.LockEprom(sts_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return f"Limit cahchanged to {limit & 0xFFFF}, but LockEprom failed: comm={self.getTxRxResult(sts_comm_result)}, err={self.getRxPacketError(sts_error)}"
        return None


    def LockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_LOCK, 1)

    def unLockEprom(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_LOCK, 0)


    def WritePos(self, sts_id, pos):
        txpacket = [self.sts_lobyte(pos), self.sts_hibyte(pos)]
        sts_comm_result, sts_errpr = self.writeTxRx(sts_id, STS_GOAL_POSITION_L, len(txpacket), txpacket)
        if sts_comm_result == 0 and sts_errpr == 0:
            return True
        else:
            return None
    
    def ReadPos(self, sts_id):
        sts_present_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        if sts_comm_result == 0 and sts_error == 0:
            return sts_present_position
        else:
            return None

    def ReadAbsPos(self, sts_id):
        sts_absolute_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_ABSPOS)
        if sts_comm_result == 0 and sts_error == 0:
            return sts_absolute_position
        else:
            return None
        
    def WritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def send_goal(self, sid, goal_pos, speed, acc,
                last_sent=[None],  # tiny cache to avoid duplicate sends
                tol=1):
        goal_pos &= 0xFFFF
        speed    &= 0xFFFF
        acc      &= 0xFF

        prev = last_sent[0]
        if prev and (abs(((goal_pos - prev[0] + 0x8000) & 0xFFFF) - 0x8000) <= tol
                    and prev[1] == speed and prev[2] == acc):
            return True  # nothing changed enough to resend

        sts_comm_result, sts_error = self.WritePosEx(sid, goal_pos, speed, acc)
        if sts_comm_result == COMM_SUCCESS and sts_error == 0:
            last_sent[0] = (goal_pos, speed, acc)
            return True
        return None
   

    def WritePosExOff(self, sts_id, position, speed, acc):
        txpacket = [acc,
                    self.sts_lobyte(position),
                    self.sts_hibyte(position),
                    0, 0,
                    self.sts_lobyte(speed),
                    self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)
    
    def WriteSignedPosEx(self, sts_id, signed_position, speed, acc):
        position = signed_position & 0xFFFF  # 2's complement conversion
        txpacket = [acc, position & 0xFF, (position >> 8) & 0xFF, 0, 0, speed & 0xFF, (speed >> 8) & 0xFF]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    # def ReadPos(self, sts_id):
    #     sts_present_position, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
    #     return self.sts_tohost(sts_present_position, 15), sts_comm_result, sts_error

    def ReadSpeed(self, sts_id):
        sts_present_speed, sts_comm_result, sts_error = self.read2ByteTxRx(sts_id, STS_PRESENT_SPEED_L)
        return self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadPosSpeed(self, sts_id):
        sts_present_position_speed, sts_comm_result, sts_error = self.read4ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        sts_present_position = self.sts_loword(sts_present_position_speed)
        sts_present_speed = self.sts_hiword(sts_present_position_speed)
        return self.sts_tohost(sts_present_position, 15), self.sts_tohost(sts_present_speed, 15), sts_comm_result, sts_error

    def ReadPosSpeedAccCurrent(self, sts_id):
        # Read 4 bytes: [Pos_L, Pos_H, Speed_L, Speed_H] starting at STS_PRESENT_POSITION_L
        pos_speed_32, comm1, err1 = self.read4ByteTxRx(sts_id, STS_PRESENT_POSITION_L)
        pos_raw = self.sts_loword(pos_speed_32)
        speed_raw = self.sts_hiword(pos_speed_32)

        # Convert to host signed (15-bit)
        pos = self.sts_tohost(pos_raw, 15)
        speed = self.sts_tohost(speed_raw, 15)

        # Read ACC from SRAM (0x29 / STS_ACC)
        acc, comm2, err2 = self.read1ByteTxRx(sts_id, STS_ACC)

        # Read current (2bytes: 69, 70)
        current_raw, comm3, err3 = self.read2ByteTxRx(sts_id, STS_PRESENT_CURRENT_L)
        current = self.sts_tohost(current_raw,15)

        # Prefer to surface any error from either read
        comm_result = next((c for c in (comm1, comm2, comm3) if c != 0), 0)
        sts_error = next((e for e in (err1, err2, err3) if e != 0), 0)

        return pos, speed, acc,current, comm_result, sts_error


    def ReadMoving(self, sts_id):
        moving, sts_comm_result, sts_error = self.read1ByteTxRx(sts_id, STS_MOVING)
        return moving, sts_comm_result, sts_error

    def SyncWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.groupSyncWrite.addParam(sts_id, txpacket)

    def RegWritePosEx(self, sts_id, position, speed, acc):
        txpacket = [acc, self.sts_lobyte(position), self.sts_hibyte(position), 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.regWriteTxRx(sts_id, STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, sts_id):
        return self.write1ByteTxRx(sts_id, STS_MODE, 1)

    def WriteSpec(self, sts_id, speed, acc):
        speed = self.sts_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.sts_lobyte(speed), self.sts_hibyte(speed)]
        return self.writeTxRx(sts_id, STS_ACC, len(txpacket), txpacket)
