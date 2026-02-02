struct trap_vel
{
    // Estado inicial/final
    double q0{0.0}, qf{0.0};
    double v0{0.0}, vf{0.0};

    // Signo de movimiento (+1 si qf>=q0, -1 si qf<q0)
    double s{+1.0};

    // Parámetros del perfil
    double a{0.0};      // módulo de aceleración
    double vp{0.0};     // velocidad pico alcanzada (<= v_max)
    double tf{0.0};     // tiempo total

    // Tiempos de tramo
    double t1{0.0};     // aceleración (v0 -> vp)
    double tc{0.0};     // constante (vp)
    double t2{0.0};     // deceleración (vp -> vf)

    // Puntos de cambio (posiciones)
    double q1{0.0};     // posición al final de t1
    double q2{0.0};     // posición al final de t1+tc

    // Flags útiles
    bool has_cruise{false}; // tc>0
    bool feasible{true};    // si cumple exactamente con tf y D (dentro de tolerancia)
};




#include <cmath>
#include <algorithm>

extern double v_max; // global
extern double t_a;   // global (ojo: en tu planteamiento original: a = v_max/t_a)

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}

trap_vel calcular_coeficientes(double q0, double qf, double v0, double vf, double tf)
{
    trap_vel c;
    c.q0 = q0; c.qf = qf;
    c.tf = tf;

    // Signo de movimiento
    c.s = (qf >= q0) ? +1.0 : -1.0;

    // Pasamos a "magnitudes" en el sentido del movimiento
    const double D  = std::abs(qf - q0);
    const double v0m = std::abs(v0); // si ya vienen con signo coherente, puedes usar v0*c.s, etc.
    const double vfm = std::abs(vf);

    c.v0 = v0m;
    c.vf = vfm;

    // Aceleración fija por tus globales
    // (si quieres que dependa de vf: a=(v_max-vf)/t_a, cámbialo aquí)
    c.a = v_max / t_a;
    const double a = c.a;

    // Checks básicos
    const double eps = 1e-9;
    if (tf <= eps || a <= eps) {
        c.feasible = false;
        return c;
    }

    // Si v0 o vf superan v_max, no cabe en el perfil clásico sin empezar frenando, etc.
    // Lo marco como "no factible" (puedes decidir saturar o permitirlo).
    if (v0m > v_max + 1e-9 || vfm > v_max + 1e-9) {
        c.feasible = false;
        // seguimos para que al menos devuelva algo razonable si quieres
    }

    // 1) Resolver vp con tf y D usando la ecuación general (permitiendo tc>=0).
    // Cuadrática: vp^2 - B*vp + C = 0
    const double B = a*tf + v0m + vfm;
    const double C = a*D + 0.5*(v0m*v0m + vfm*vfm);

    const double disc = B*B - 4.0*C;
    if (disc < -1e-9) {
        c.feasible = false;
        return c;
    }
    const double sqrt_disc = std::sqrt(std::max(0.0, disc));

    // Elegimos la raíz "pequeña" => vp más bajo (la que suele dar tc>=0 para tiempos grandes)
    double vp = 0.5*(B - sqrt_disc);

    // Si por algún motivo sale negativa, probamos la otra
    if (vp < 0.0) vp = 0.5*(B + sqrt_disc);

    // Respetar límite de velocidad máxima
    vp = clamp(vp, 0.0, v_max);
    c.vp = vp;

    // 2) Con vp, calculamos tiempos
    c.t1 = (vp - v0m) / a;
    c.t2 = (vp - vfm) / a;

    // Si vp < v0 o vp < vf, esto implicaría empezar frenando en ese tramo. Marcamos infeasible.
    if (c.t1 < -1e-9 || c.t2 < -1e-9) {
        c.feasible = false;
    }

    // tc por tiempo
    c.tc = tf - c.t1 - c.t2;

    // ¿hay meseta?
    c.has_cruise = (c.tc > 1e-9);

    // Si tc ~ 0 => triangular
    if (c.tc < 0.0) {
        // Con el vp "clampado" puede ocurrir que no cuadre tf.
        // Lo marcamos como no factible.
        c.feasible = false;
        c.tc = 0.0;
    }

    // 3) Precalcular posiciones q1, q2 para la evaluación
    // Distancia en tramo 1:
    const double x1 = 0.5*(v0m + vp)*c.t1;           // trapecio
    const double xc = vp * c.tc;
    c.q1 = q0 + c.s * x1;
    c.q2 = c.q1 + c.s * xc;

    // 4) Chequeo final de consistencia en distancia (opcional pero útil)
    // Distancia tramo 2:
    const double x2 = 0.5*(vp + vfm)*c.t2;
    const double D_pred = x1 + xc + x2;
    if (std::abs(D_pred - D) > 1e-6) {
        c.feasible = false;
    }

    return c;
}



#include <std_msgs/msg/float64_multi_array.hpp>

std_msgs::msg::Float64MultiArray eval_trap(const trap_vel &c, double t)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(5);

    // Clamp del tiempo
    double tt = std::max(0.0, std::min(t, c.tf));

    double q= c.q0;
    double v= c.s * c.v0;   // con signo
    double a= 0.0;

    const double s = c.s;
    const double A = c.a;

    if (tt <= c.t1) {
        // Tramo 1: acelera v0 -> vp con +a
        a = s * A;
        v = s * (c.v0 + A*tt);
        q = c.q0 + s * (c.v0*tt + 0.5*A*tt*tt);
    }
    else if (tt <= c.t1 + c.tc) {
        // Tramo 2: constante a vp
        const double tau = tt - c.t1;
        a = 0.0;
        v = s * c.vp;
        q = c.q1 + s * (c.vp * tau);
    }
    else {
        // Tramo 3: desacelera vp -> vf con -a
        const double tau = tt - (c.t1 + c.tc);
        a = -s * A;
        v = s * (c.vp - A*tau);
        q = c.q2 + s * (c.vp*tau - 0.5*A*tau*tau);
    }

    msg.data[0] = q;
    msg.data[1] = v;
    msg.data[2] = a;
    msg.data[3] = 0.0;                         // jerk 0 (por tramos)
    msg.data[4] = (tt / c.tf) * 100.0;

    return msg;
}




