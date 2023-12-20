# Communication standards for EDR RCJ2024
## Vision to Navigation
Communication method: Socket communication

Port: 4242  
Bidirectional: Yes (?)

Information that the protocol should be able to transmit:

- Victim type (including none)
    - Also includes potential victims
- Victim position
- Timestamp for captured image frame

If victim is direclty right/left, first send potential victim, with position and timestamp. Then navigation slows down or stops and updates the slowing/stopping status to the vision program. Then a normal victim type may be sent, the side, position and timestamp.

If victim is in front or out to the sides (for example in adjacent tile), send an unconfirmed victim type, side, position(?) (different from potition above) and timestamp. Then it is up to global navigation to decide wether to investigate or not. Then normal victim handling takes over. No confirmations are sent.

### Concrete implementation
Start char: '!'  
Delimiter: ','

Message types:
- Victim detection: 'v'
- Confirmation: 'c'


Victim detection message structure: !v,\<detection type>,\<victim type>,\<camera>,\<position>,\<timestamp>

- Detection type: 'p'/'c'
- Victim type: 'g'/'y'/'r'/'u'/'s'/'h'
- Camera: Integer staring from 0, incrementing by 1. Fusion program determines left/right
- Position:
    - \<x-coord>,\<y-coord>
    - Integer in mm from the camera center of FOV. Positive direction is normal math coordinate system (positive right-up)
- Timestamp: Unix millisecond timestamp

Confirmation message structure: !c


## Navigation threads
Communication method: Shared variables

Data that should be shared:
- Local robot pose
- Global robot pose
- Array/vector/queue of commands for local to execute
- Fusion -> (global) -> local: detection slowdown
- Fusion -> global -> local: kit drop interrupt
    - Read what to drop and where from other variable
- speed setting (slow, normal, turbo)
- Ground colour data from fusion
- Ramp data: direction, x-length, y-length, (length)
- Väggar
- Obstacles?
- Bump sensor