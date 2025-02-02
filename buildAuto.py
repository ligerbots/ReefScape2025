from sys import argv

# test script to print an outline of auto routine, demonstrating where the parameters fit in the sequence
# no actual paths are touched, just a POC script

print(argv)

start  = argv[1]
source = argv[2]

(score1, score2, score3, score4) = (f'Reef{argv[3]}', f'Reef{argv[4]}', f'Reef{argv[5]}', f'Reef{argv[6]}')

auto = f'''

FollowPath: {start} to {score1}
Score Corral
FollowPath: {score1} to {source}
Intake Corral
FollowPath: {source} to {score2}
Score Corral
FollowPath: {score2} to {source}
Intake Corral
FollowPath: {source} to {score3}
Score Corral
FollowPath: {score3} to {source}
Intake Corral
FollowPath: {source} to {score4}
Score Corral

'''

print(auto)
