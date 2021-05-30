from utils import Utils

u=Utils()


def gcal(g, alpha, beta, theta, phi):
    g1 = g * (-u.cos(alpha) * u.cos(beta) * u.sin(theta) + u.sin(beta) * u.cos(theta) * u.sin(
        phi) + u.sin(alpha) * u.cos(beta) * u.cos(theta) * u.cos(phi))
    g2 = g * (u.cos(alpha) * u.cos(theta) * u.cos(phi) + u.sin(alpha) * u.sin(theta))
    g3 = g * (u.cos(beta) * u.cos(theta) * u.sin(phi) + u.sin(beta) * u.cos(alpha) * u.sin(
        theta) - u.sin(alpha) * u.sin(beta) * u.cos(theta) * u.cos(phi))
    return [g1, g2, g3]


def NonlinearModel(states, U, Nsolver, dt):
    # states
    Vt = states[0]
    alpha = states[1]
    beta = states[2]
    phi = states[3]
    theta = states[4]
    psi = states[5]
    p = states[6]
    q = states[7]
    r = states[8]
    Ft = states[9]

    # controller outputs
    u1 = U[0]
    u2 = U[1]
    u3 = U[2]
    ut = U[3]

    # ModelParameters
    S = u.ft2Tom2(184)
    m = u.slugTokg(85.4)
    rho = u.slugoverft3Tokgoverm3(0.002378)
    g = 9.81

    # Aerodynamic coefficients
    CLo = 0.41
    CLalpha = 4.44
    CDo = 0.05
    CDalpha = 0.33
    Cybeta = -0.564

    q2 = u.qCalc(rho, Vt)
    L = q2 * S * u.CLcal(CLo, CLalpha, alpha)
    D = q2 * S * u.CDcal(CDo, CDalpha, alpha)
    Y = q2 * S * u.Cycal(Cybeta, beta)

    g1, g2, g3 = gcal(g, alpha, beta, theta, phi)
    
    u_vel=Vt*u.cos(alpha)*u.cos(beta)
    v_vel=Vt*u.sin(alpha)
    w_vel=Vt*u.sin(alpha)*u.cos(beta)     

    for i in range(Nsolver):
        state_dot = [(-D + Ft * u.cos(alpha) * u.cos(beta) + m * g1) / m,  # V_dot
                     q - (p * u.cos(alpha) + r * u.sin(alpha)) * u.tan(beta) + (
                                 -L - Ft * u.sin(alpha) + m * g2) / (m * Vt * u.cos(beta)),  # alpha_dot
                     p * u.sin(alpha) - r * u.cos(alpha) + (
                                 Y - Ft * u.cos(alpha) * u.sin(beta) + m * g3) / (m * Vt),  # beta_dot
                     p + q * u.sin(phi) * u.tan(theta) + r * u.cos(phi) * u.tan(theta),  # phi_dot
                     q * u.cos(phi) - r * u.sin(phi),  # theta_dot
                     q * u.sin(phi) / u.cos(theta) + r * u.cos(phi) / u.cos(theta),  # psi_dot
                     u1,  # p_dot
                     u2,  # q_dot
                     u3,  # r_dot
                     ut,  # Ft_dot
                     u_vel*u.cos(theta)*u.cos(psi)+v_vel*(u.sin(phi)*u.sin(theta)*u.cos(psi)-u.cos(phi)*u.sin(psi))+w_vel*(u.cos(phi)*u.sin(theta)*u.cos(psi)+u.sin(phi)*u.sin(psi)), #x_dot
                     u_vel*u.cos(theta)*u.sin(psi)+v_vel*(u.sin(phi)*u.sin(theta)*u.sin(psi)+u.cos(phi)*u.cos(psi))+w_vel*(u.cos(phi)*u.sin(theta)*u.cos(psi)-u.sin(phi)*u.cos(psi)), #y_dot
                     u_vel*u.sin(theta)-v_vel*u.sin(phi)*u.cos(theta)-w_vel*u.cos(phi)*u.cos(theta)     #z_dot
                     ]

        for j in range(len(states)):
            states[j] = states[j] + state_dot[j] * dt
    if(states[9]<0):
        states[9]=0
    return states