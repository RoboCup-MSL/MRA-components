# A player in the game, typically a robot or human participant

int32 id          # T-shirt number
bool active       # Participating in the game (inactive if figuring out position)
bool human        # Indicates if the player is a human
geometry_msgs/Pose position  # Current position of the player
geometry_msgs/Twist velocity  # Current velocity of the player
bool hasball      # Indicates if the player has possession of the ball

# Future considerations:
# - double energy: Represents the player's energy level
# - double competence: Represents the player's skill level
# - radius: Represents the player's effective radius
