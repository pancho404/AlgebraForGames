using System;
using UnityEngine;

namespace CustomMath
{
    public struct Quat : IEquatable<Quat>
    {
        #region Variables

        public float x;
        public float y;
        public float z;
        public float w;

        #endregion

        #region Constants

        public const float kEpsilon = 1E-06F;

        #endregion

        #region Default Values

        public static Quat Identity => new Quat(0, 0, 0, 1);

        #endregion

        #region Constructors

        public Quat(float x, float y, float z, float w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        #endregion

        #region Operators

        public static bool operator ==(Quat lhs, Quat rhs) => IsEqualUsingDot(Dot(lhs, rhs));

        public static bool operator !=(Quat lhs, Quat rhs) => !(lhs == rhs);

        /// <summary>
        /// Multiplicacion de <see cref="Quat"/>
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Quat operator *(Quat lhs, Quat rhs)
        {
            float w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z; // Real
            float x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y; // imaginario I
            float y = lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z; // imaginario J
            float z = lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x; // imaginario K

            return new Quat(x, y, z, w); // Choclo final xD
        }

        /// <summary>
        /// Multiplica el <see cref="Quat"/> con el <see cref="Vec3"/>
        /// y devuelve una copia del <see cref="Vec3"/> con la rotacion del <see cref="Quat"/>
        /// </summary>
        /// <param name="rotation"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static Vec3 operator *(Quat rotation, Vec3 point)
        {
            //Eje de rotacion principal
            float rotX = rotation.x * 2f;
            float rotY = rotation.y * 2f;
            float rotZ = rotation.z * 2f;

            
            float rotX2 = rotation.x * rotX;
            float rotY2 = rotation.y * rotY;
            float rotZ2 = rotation.z * rotZ;

            //Planos de Rotacion
            float rotXY = rotation.x * rotY;
            float rotXZ = rotation.x * rotZ;
            float rotYZ = rotation.y * rotZ;

            //Parte Real
            float rotWX = rotation.w * rotX;
            float rotWY = rotation.w * rotY;
            float rotWZ = rotation.w * rotZ;

            Vec3 result = Vec3.Zero;

            //Se rotan los ejes y se ignora uno de los ejes, dependiendo de que queramos sacar y se generan los unitarios
            result.x = (1f - (rotY2 + rotZ2)) * point.x + (rotXY - rotWZ) * point.y + (rotXZ + rotWY) * point.z; //Se ignora X
            result.y = (rotXY + rotWZ) * point.x + (1f - (rotX2 + rotZ2)) * point.y + (rotYZ - rotWX) * point.z; //Se ignora Y
            result.z = (rotXZ - rotWY) * point.x + (rotYZ + rotWX) * point.y + (1f - (rotX2 + rotY2)) * point.z; //Se ignora Z

            return result;
        }

        public static implicit operator Quaternion(Quat quat)
        {
            return new Quaternion(quat.x, quat.y, quat.z, quat.w);
        }

        public static implicit operator Quat(Quaternion quaternion)
        {
            return new Quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        }

        #endregion

        #region Functions

        /// <summary>
        /// Devuelve los angulos de euler de un <see cref="Quat"/>
        /// y tambien se le puede asignar un <see cref="Vec3"/> como angulos
        /// </summary>
        public Vec3 EulerAngles
        {
            get => ToEulerAngles(this) * Mathf.Rad2Deg;

            set => this = ToQuaternion(value * Mathf.Deg2Rad);
        }

        /// <summary>
        /// Devuelve una copia del Quat ya normalizado.
        /// </summary>
        public Quat Normalized => Normalize(this);

        public static Quat Euler(float x, float y, float z) => ToQuaternion(new Vec3(x, y, z) * Mathf.Deg2Rad);

        public static Quat Euler(Vec3 euler) => ToQuaternion(euler);

        /// <summary>
        /// Transforma el <see cref="Vec3"/> en un <see cref="Quat"/>.
        /// </summary>
        /// <param name="vec3"></param>
        /// <returns></returns>
        private static Quat ToQuaternion(Vec3 vec3) // yaw (Z), pitch (Y), roll (X)
        {
            //Se calculan las rotaciones de cada eje (Coseno para la parte real y seno para la parte imaginaria (X,Y,Z)
            float cz = Mathf.Cos(Mathf.Deg2Rad * vec3.z / 2);
            float sz = Mathf.Sin(Mathf.Deg2Rad * vec3.z / 2);
            float cy = Mathf.Cos(Mathf.Deg2Rad * vec3.y / 2);
            float sy = Mathf.Sin(Mathf.Deg2Rad * vec3.y / 2);
            float cx = Mathf.Cos(Mathf.Deg2Rad * vec3.x / 2);
            float sx = Mathf.Sin(Mathf.Deg2Rad * vec3.x / 2);

            Quat quat = new Quat(); // Se crea el quaternion y se le asignan las rotaciones (abajo)

            //Se trabaja todo en un quaternion para evitar el desfasaje por la perdida de precision de punto flotante
            quat.w = cx * cy * cz + sx * sy * sz;   // Real
            quat.x = sx * cy * cz - cx * sy * sz;   // Imaginario X
            quat.y = cx * sy * cz + sx * cy * sz;   // Imaginario Y
            quat.z = cx * cy * sz - sx * sy * cz;   // Imaginario Z

            return quat;
        }

        /// <summary>
        /// Transforma el <see cref="Quat"/> en un <see cref="Vec3"/>.
        /// (En radianes)
        /// </summary>
        /// <param name="quat"></param>
        /// <returns></returns>
        private static Vec3 ToEulerAngles(Quat quat)
        {
            float sqw = quat.w * quat.w;
            float sqx = quat.x * quat.x;
            float sqy = quat.y * quat.y;
            float sqz = quat.z * quat.z;
            float unit = sqx + sqy + sqz + sqw; 
            float test = quat.x * quat.w - quat.y * quat.z; //se crea esta variable para que obtengamos el valor de X
            Vec3 vec3;

            //dependiendo del valor de X podemos obtener 2 singularidades y se le sumara o restara Pi para anular esta singularidad
            if (test > 0.4999f * unit) // singularidad en polo norte
            {
                vec3.y = 2f * Mathf.Atan2(quat.y, quat.x);
                vec3.x = Mathf.PI / 2;
                vec3.z = 0; //Se asigna 0 a Z para asegurarnos de que X y Z no se fusionen
            }
            if (test > -0.4999f * unit) //singularidad en polo sur
            {
                vec3.y = -2f * Mathf.Atan2(quat.y, quat.x);
                vec3.x = -Mathf.PI / 2;
                vec3.z = 0; //Se asigna 0 a Z para asegurarnos de que X y Z no se fusionen
            }


            //Se desarma la matriz de rotacion para darnos los valores de cada eje en Eulers, asi los podemos asignar a lso ejes del Vec3.
            //Al quat le pasamos los datos del contructor en desorden porq en Unity las rotaciones se aplican en otro orden comparado con la formula de wikipedia (la de abajo)
            Quat q = new Quat(quat.w, quat.z, quat.x, quat.y);
            vec3.y = (float)Math.Atan2(2f * q.x * q.w + 2f * q.y * q.z, 1 - 2f * (q.z * q.z + q.w * q.w)); //Yaw
            vec3.x = (float)Math.Asin(2f * (q.x * q.z - q.w * q.y));                                       //Pitch
            vec3.z = (float)Math.Atan2(2f * q.x * q.y + 2f * q.z * q.w, 1 - 2f * (q.y * q.y + q.z * q.z)); //Roll
            return Vec3.Normalize(vec3);
        }


        /// <summary>
        /// Invierte la rotacion del quaternion.
        /// </summary>
        /// <param name="rotation">Quaternion que queremos invertir.</param>
        /// <returns></returns>
        public static Quat Inverse(Quat rotation)
        {
            Quat q;
            q.w = rotation.w;
            q.x = -rotation.x;
            q.y = -rotation.y;
            q.z = -rotation.z;
            return q;
        }

        /// <summary>
        /// Devuelve un <see cref="Quat"/> normalizado.
        /// </summary>
        /// <param name="quat">Quaternion que queremos normalizar.</param>
        /// <returns></returns>
        public static Quat Normalize(Quat quat)
        {
            float sqrtDot = Mathf.Sqrt(Dot(quat, quat));

            if (sqrtDot < Mathf.Epsilon)
            {
                return Identity;
            }

            return new Quat(quat.x / sqrtDot, quat.y / sqrtDot, quat.z / sqrtDot, quat.w / sqrtDot);
        }

        /// <summary>
        /// Normaliza el Quat.
        /// </summary>
        public void Normalize() => this = Normalize(this);

        public static Quat Lerp(Quat a, Quat b, float t) => LerpUnclamped(a, b, Mathf.Clamp01(t));

        public static Quat LerpUnclamped(Quat a, Quat b, float t)
        {
            Quat r;
            float time = 1 - t;

            //Multiplicamos el tiempo restante por el eje correspondiente del primer cuaternion y luego sumamos al producto entre el tiempo y el eje correspondiendte del segundo cuaternion
            r.x = time * a.x + t * b.x;
            r.y = time * a.y + t * b.y;
            r.z = time * a.z + t * b.z;
            r.w = time * a.w + t * b.w;

            r.Normalize();

            return r;
        }


        /// <summary>
        /// Interpola esféricamente entre los <see cref="Quat"/> a y b por t. El parámetro t está sujeto al rango [0, 1].
        // https://www.youtube.com/watch?v=dttFiVn0rvc&list=PLW3Zl3wyJwWNWsJIPZrmY19urkYHXOH3N
        // https://en.wikipedia.org/wiki/Slerp#:~:text=Quaternion%20Slerp,-When%20Slerp%20is&text=The%20effect%20is%20a%20rotation,of%20unit%20quaternions%2C%20S3
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Quat Slerp(Quat a, Quat b, float t) => SlerpUnclamped(a, b, Mathf.Clamp01(t));

        /// <summary>
        /// Interpola esféricamente entre a y b por t. El parámetro t no está sujeto.
        /// https://splines.readthedocs.io/en/latest/rotation/slerp.html
        /// https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static Quat SlerpUnclamped(Quat a, Quat b, float t)
        {
            Quat r;

            float time = 1 - t;

            float wa, wb;

            float theta = Mathf.Acos(Dot(a, b)); // El resultado del arco coseno del producto punto entre 2 Quaterniones, representa el sentido.

            if (theta < 0) theta = -theta; // Si el resultado es menor a cero, se invierte el valor para que siempre sea un numero positivo.

            float sn = Mathf.Sin(theta);

            wa = Mathf.Sin(time * theta) / sn;  //Angulo q ya se recorrio del 1er cuaternion a r
            wb = Mathf.Sin((1 - time) * theta) / sn; //Angulo que falta recorrer de r a segundo quat

            r.x = wa * a.x + wb * b.x;
            r.y = wa * a.y + wb * b.y;
            r.z = wa * a.z + wb * b.z;
            r.w = wa * a.w + wb * b.w;

            r.Normalize();

            return r;
        }

        /// <summary>
        /// Devuelve el angulo entre 2 <see cref="Quat"/> en grados.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float Angle(Quat a, Quat b)
        {
            // Se calcula el producto punto para saber si los quaterniones tienen la misma orientacion, si la tienen entonces el angulo es 0.
            float dot = Dot(a, b);

            // Se busca el numero mas chico entre el absoluto del producto punto y 1.
            // Cuando se consigue eso se calcula el arco coseno en radianes.
            // Se realizan las multiplicaciones para conseguir el angulo en grados.

            return IsEqualUsingDot(dot) ? 0f : (Mathf.Acos(Mathf.Min(Mathf.Abs(dot), 1f)) * 2f * Mathf.Rad2Deg); //1 es 180 grados en radianes (el maximo angulo posobile entre dos quaternioens)
        }

        //SI el producto punto es un numero muy cercano a 1, son iguales (nunca da 1 exacto por la precision
        private static bool IsEqualUsingDot(float dot) => dot > 0.999999f; // uso este numero constante para darle un margen a la precicion flotante.

        /// <summary>
        /// Devuelve el producto Punto entre 2 <see cref="Quat"/>.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float Dot(Quat a, Quat b) => a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

        // From https://stackoverflow.com/questions/12435671/quaternion-lookat-function
        public static Quat LookRotation(Vec3 forward, Vec3 upwards)
        {
            Vec3 dir = Vec3.Normalize(upwards - forward); //Se obtiene la direccion mediante la obtencion de un nuevo vector resultante de la resta de los dos vectores parametro
            Vec3 rotAxis = Vec3.Cross(Vec3.Forward, dir); //Se utiliza el producto cruz para obtener el eje de rotacion.
            float dot = Vec3.Dot(Vec3.Forward, dir); //Se obtiene el producto punto entre el Forward y el vector de direccion

            //Se asignan los valores
            Quat result;
            result.x = rotAxis.x;
            result.y = rotAxis.y;
            result.z = rotAxis.z;
            result.w = dot + 1;

            //Se devuelve el vector normalizado
            return result.Normalized;
        }

        public static Quat LookRotation(Vec3 forward) => LookRotation(forward, Vec3.Up);

        public static Quat RotateTowards(Quat from, Quat to, float maxDegreesDelta)
        {
            float angle = Angle(from, to);

            if (angle == 0f)
            {
                return to;
            }

            return SlerpUnclamped(from, to, Mathf.Min(1f, maxDegreesDelta / angle));
        }
      
        #endregion

        #region Internals

        public override bool Equals(object other)
        {
            if (!(other is Quat))
            {
                return false;
            }

            return Equals((Quat)other);
        }

        public bool Equals(Quat other)
        {
            return x.Equals(other.x) &&
                   y.Equals(other.y) &&
                   z.Equals(other.z) &&
                   w.Equals(other.w);
        }

        public override string ToString() => $"({x:0.0},{y:0.0},{z:0.0},{w:0.0})";

        public override int GetHashCode() => x.GetHashCode() ^ (y.GetHashCode() << 2) ^ (z.GetHashCode() >> 2) ^ (w.GetHashCode() >> 1);
        #endregion
    }
}