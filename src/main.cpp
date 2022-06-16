#include <iostream>
#include <complex>
#include <cmath>
using namespace std::complex_literals;

struct Vector3f
{
    float x = .0f;
    float y = .0f;
    float z = .0f;

    Vector3f(float x, float y, float z)
        :x(x), y(y), z(z){}

    Vector3f operator+(const Vector3f& rhb) const
    {
        return { x + rhb.x, y + rhb.y, z + rhb.z };
    }

    Vector3f operator-(const Vector3f& rhb) const
    {
        return { x - rhb.x, y - rhb.y, z - rhb.z };
    }

    Vector3f operator*(const float d) const
    {
        return { x*d, y*d, z*d };
    }

    Vector3f operator/(const float d) const
    {
        return { x/d, y/d, z/d };
    }

    Vector3f& operator+=(const Vector3f& rhb)
    {
        x += rhb.x;
        y += rhb.y;
        z += rhb.x;
        return *this;
    }

    Vector3f& operator*=(const float d)
    {
        x *= d;
        y *= d;
        z *= d;
        return *this;
    }

    Vector3f cross(const Vector3f& rhb) const
    {
        return {
            (y*rhb.z - z*rhb.y), 
            (z*rhb.x - x*rhb.z), 
            (x*rhb.y - y*rhb.x)
            };
    }

    float dot(const Vector3f& rhb) const
    {
        return x*rhb.x + y*rhb.y + z*rhb.z;
    }

    float length() const
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3f normalize() const
    {
        float d = std::sqrt((x * x) + (y * y) + (z * z));
        return *this / d;
    }
    
    float angleBetween(const Vector3f& rhb) const
    {
        return std::acos((dot(rhb)/rhb.length()) / length());
    }

    friend
    std::ostream& operator<<(std::ostream& os, const Vector3f& vec)
    {
        return os << "[ " << vec.x << "; " << vec.y << "; " << vec.z << " ]";
    }
};


struct Quaternion
{
    float w = .0f;
    union 
    {
        Vector3f v = { 0, 0, 0 };
        struct 
        {
            float x;
            float y;
            float z;
        };
        
    };

    Quaternion(float w, float x, float y, float z)
        :x(x), y(y), z(z), w(w){}

    Quaternion(float rad, Vector3f vec)
    {
        w = std::cos(rad);
        vec *= std::sin(rad);
        x = vec.x;
        y = vec.y;
        z = vec.z;
    }

    Quaternion operator+(const Quaternion& rhb) const
    {
        return { w + rhb.w, x + rhb.x, y + rhb.y, z + rhb.z };
    }

    Quaternion operator-(const Quaternion& rhb) const
    {
        return { w - rhb.w, x - rhb.x, y - rhb.y, z - rhb.z };
    }

    Quaternion operator+(const float d) const
    {
        return { w + d, x, y, z };
    }
    
    Quaternion operator-(const float d) const
    {
        return { w - d, x, y, z};
    }

    Quaternion operator*(const float d) const
    {
        return { w*d, x*d, y*d, z*d };
    }

    Quaternion operator*(const Quaternion& rhb) const
    {
        return {
            ( w*rhb.w - x*rhb.x - y*rhb.y - z*rhb.z ),
            ( w*rhb.x + x*rhb.w + y*rhb.z - z*rhb.y ),
            ( w*rhb.y - x*rhb.z + y*rhb.w + z*rhb.x ),
            ( w*rhb.z + x*rhb.y - y*rhb.x + z*rhb.w ),
            };
    }

    Quaternion operator/(const float d) const
    {
        return { w/d, x/d, y/d, z/d };
    }
    
    Quaternion hamilton(const Quaternion& rhb) const
    {
        return {
            ( w*rhb.w - x*rhb.x - y*rhb.y - z*rhb.z ),
            ( w*rhb.x + x*rhb.w + y*rhb.z - z*rhb.y ),
            ( w*rhb.y - x*rhb.z + y*rhb.w + z*rhb.x ),
            ( w*rhb.z + x*rhb.y - y*rhb.x + z*rhb.w ),
            };
    }



/*
    Quaternion cross(const Quaternion& rhb) const
    {
        return {
            (y*rhb.z - z*rhb.y), 
            (z*rhb.x - x*rhb.z), 
            (x*rhb.y - y*rhb.x)
            };
    }

    float dot(const Quaternion& rhb) const
    {
        return x*rhb.x + y*rhb.y + z*rhb.z;
    }

    float length() const
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    Quaternion normalize(const Quaternion& vec) const
    {
        float d = std::sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
        return vec / d;
    }
    
    float angleBetween(const Quaternion& rhb) const
    {
        return std::acos((dot(rhb)/rhb.length()) / length());
    }
*/

    float getScalarPart() const
    {
        return w;
    }

    Vector3f getVectorPart() const
    {
        return v;
    }
    
    friend
    std::ostream& operator<<(std::ostream& os, const Quaternion& vec)
    {
        return os << "[ " << vec.w << "; " << vec.x << "i; " << vec.y << "j; " << vec.z << "k ]";
    }

};

namespace
{
constexpr float radMultiplier = 180.0f / M_PI;

constexpr
inline
float radToDeg(float rad)
{
    return rad * radMultiplier;
}

constexpr
inline
float degToRad(float deg)
{
    return deg / radMultiplier;
}
}


int main()
{
    
    float angle = degToRad(45.0f);
    Quaternion p = Quaternion( 0, 0, 1, 1);
    Quaternion q1 = Quaternion( angle, Vector3f{ 1, 0, 0 }.normalize() );
    Quaternion q2 = Quaternion( -angle, { 1, 0, 0 } );

    std::cout << p << " 90deg > ";

    p = q1 * p * q2;
    
    std::cout << p << "\n";
    

    return 0;
}
