import math

l1 = 6
l2 = 6


def calculate_ik_updated(X, Z):
    dist_sq = X**2 + Z**2
    R = math.sqrt(dist_sq)
    
    if R > (l1 + l2) or R < abs(l1 - l2):
        print(f"Target ({X}, {Z}) is out of reach!")
        return None
    if R == 0:
        return None
        
    D = (dist_sq + l1**2 - l2**2) / (2 * l1 * R)
    
    gamma = math.atan2(X, Z)
    
    alpha1 = gamma + math.acos(D)
    alpha2 = gamma - math.acos(D)
    
    beta1 = alpha1 + math.atan2(X - l1 * math.sin(alpha1), l1 * math.cos(alpha1) - Z)
    beta2 = alpha2 + math.atan2(X - l1 * math.sin(alpha2), l1 * math.cos(alpha2) - Z)
    
    def normalize(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
        
    return (
        (normalize(alpha1), normalize(beta1)), 
        (normalize(alpha2), normalize(beta2))
    )

def verify_new_fk(alpha, beta):
    """Plugs angles back into your NEW FK equations to verify."""
    X_check = l1 * math.sin(alpha) + l2 * math.sin(beta - alpha)
    Z_check = l1 * math.cos(alpha) - l2 * math.cos(beta - alpha)
    return X_check, Z_check

target_X = 6
target_Z = 6

print(f"--- Target: X={target_X}, Z={target_Z} ---\n")
solutions = calculate_ik_updated(target_X, target_Z)

if solutions:
    sol1, sol2 = solutions
    
    print("Solution 1:")
    print(f"  Alpha: {math.degrees(sol1[0]):.2f}°, Beta: {math.degrees(sol1[1]):.2f}°")
    check1_X, check1_Z = verify_new_fk(sol1[0], sol1[1])
    print(f"  Verification -> X: {check1_X:.2f}, Z: {check1_Z:.2f}\n")
    
    print("Solution 2:")
    print(f"  Alpha: {math.degrees(sol2[0]):.2f}°, Beta: {math.degrees(sol2[1]):.2f}°")
    check2_X, check2_Z = verify_new_fk(sol2[0], sol2[1])
    print(f"  Verification -> X: {check2_X:.2f}, Z: {check2_Z:.2f}")