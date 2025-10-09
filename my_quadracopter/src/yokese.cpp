
double g = 9.81;
double umbral_g = 0.2;

double p = 0;
double q = 0;
double r = 0;

double x_ddot = 0;
double y_ddot = 0;
double z_ddot = 0;

double integrate (double num)
{
    return(num);
}

double corregir_roll_pitch (double roll, double pitch)
{
    return(roll,pitch);         // aquí se utilizaría la aceleración
}

double corregir_yaw (double yaw)
{
    return(yaw);               // aquí se utilizaría el magnetómetro
}

double corregir_z (double z_bar, double z_ddot)
{
    double z_acel = integrate(integrate(z_ddot));

    return(z_bar+z_acel);               // aquí se utilizaría el magnetómetro
}

double roll_gir = integrate(p);
double pitch_gir = integrate(q);
double yaw_gir = integrate (r);

double roll,ptich, yaw, z;

double accel_norm = sqrt(x_ddot*x_ddot + y_ddot*y_ddot + z_ddot*z_ddot);
if (fabs(accel_norm - g) <= umbral_g)
{
    roll,pitch = corregir_roll_pitch(roll_gir,pitch_gir);
}
else
{
    roll = roll_gir;
    pitch = pitch_gir;
}

if (!isnan(R))
{
    yaw = corregir_yaw(yaw_gir);
}
else
{
    yaw = yaw_gir;
}

double z_bar = 0;

if (!isnan(R))
{
    z = corregir_z(z_bar,z_ddot);
}
else
{
    z = z_bar;
}



// COSAS PARA MAÑANA
// Acceso Máster
// Dejar los programas limpios y Publicar en Github (Mejorar el nodo action para que pueda llamrlo varias veces)
// Eliminar el Ground Truth y Estimar con sensores
// Limpiar nuevos programas y Publicar en Github
// Enviar correo al coordinador del TFG

// CUANDO TERMINE ESTO
// Ver que hago con mi vida
// Volver al curso y ponerme al día
// Ayudar a Álvaro
