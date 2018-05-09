MAX_SPEED =350

speedA = 500
speedB = 500
speedC = 600

table = []

table.append(speedA)
table.append(speedB)
table.append(speedC)
    
if max(table) == speedA:
    speedRA=MAX_SPEED
    speedRB=(speedB*MAX_SPEED)/speedA
    speedRC=(speedC*MAX_SPEED)/speedA
if max(table) == speedB:
    speedRB=MAX_SPEED
    speedRA=(speedA*MAX_SPEED)/speedB
    speedRC=(speedC*MAX_SPEED)/speedB
if max(table) == speedC:
    speedRC=MAX_SPEED
    speedRB=(speedB*MAX_SPEED)/speedC
    speedRA=(speedA*MAX_SPEED)/speedC

print('SpedA',speedRA)
print('SpedB',speedRB)
print('SpedC',speedRC)