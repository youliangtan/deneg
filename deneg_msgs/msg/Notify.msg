uint8 ERROR = 0             # Error
uint8 JOIN = 1              # Join the room
uint8 LEAVE = 2             # Leave the room
uint8 READY = 3             # Room is ready
uint8 NEGO = 4              # Negotiation round
uint8 RANK = 5              # Rank
uint8 CONSENT = 6           # Consent
uint8 ASSIGNMENT = 7        # Assignment

# The type of the message, enum above
uint8 type

# The room id
string room_id

# The name of the participant who sent the message
string source

# Generic data, depends of the type
uint8 data

# The ranking of the participants when type is RANK
string[] ranking
