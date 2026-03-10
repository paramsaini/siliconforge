import json

with open('/Users/paramsaini/Desktop/siliconforge/web_state.json') as f:
    S = json.load(f)

nl = S['netlist']
gates = nl['gates']
netMap = {g['output']: i for i, g in enumerate(gates)}
gateDepth = [0] * len(gates)
computed = [False] * len(gates)

def computeDepth(gi):
    if computed[gi]: return gateDepth[gi]
    computed[gi] = True
    maxD = 0
    for inp in gates[gi].get('inputs', []):
        if inp in netMap and netMap[inp] != gi:
            maxD = max(maxD, computeDepth(netMap[inp]) + 1)
    gateDepth[gi] = maxD
    return maxD

for i in range(len(gates)):
    computeDepth(i)

print("Depths calculated successfully:", gateDepth)
