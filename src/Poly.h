#ifndef POLY_H
#define POLY_H
#include <poly2tri.h>
#include <raylib.h>
#include <raymath.h>

#include <vector>
#include <algorithm>

using namespace std;

#define PHYSAC_PI                       3.14159265358979323846
#define PHYSAC_DEG2RAD                  (PHYSAC_PI/180.0f)
#define EPSILON	0.000001

typedef struct Tri {

    Vector2 face[2];
    Vector2 normal;

} Tri;

class Poly
{
    public:
        Poly();
        virtual ~Poly();

        std::vector<Vector2> reduce(const std::vector<Vector2>& points, const Rectangle& rect, float epsilon=2.0f);
        std::vector<Vector2> trace(const Rectangle& rect, float threshold= 0.05f);

        std::vector<Vector2> process(const Rectangle& rect, float threshold= 0.05f,float epsilon=2.0f);

        int Triangulate(const Vector2* contour,int n,Vector2* result);
        int TriangulateArray(const Vector2* contour,int n,Tri* result);



        void loadImage(Image rasterImage);


        void reset();


    protected:

    private:
    float scaleFactor;
    Color* pixels;
    bool isLoad;
    float TriangulateArea(const Vector2* contour,int n);
    bool  InsideTriangle(float Ax, float Ay, float Bx, float By,                      float Cx, float Cy,                      float Px, float Py);
    bool Snip(const  Vector2* contour,int u,int v,int w,int n,int *V);
    std::vector<Vector2> rdp(const std::vector<Vector2>& v, float optimization);
    float perpendicularDistance(const Vector2& i, const Vector2& start, const Vector2& end);
    std::vector<Vector2> marchSquare(const Rectangle& rect, const Vector2& start, float threshold);
    unsigned int getSquareValue(unsigned int x, unsigned int y, const Rectangle& rect, float threshold);
    Vector2 findFirstNoneTransparentPixel(const Rectangle& rect, float threshold);
    unsigned char getAlphaPixel(int x, int y);
    unsigned char getAlphaByPos(const Vector2& pos);
    int getIndexFromPos(unsigned int x, unsigned int y);
    unsigned char* getImageColors(Image image);
    vector<p2t::Point*> polyline;
    vector<p2t::Triangle*> triangles;
    int width;
    int height;
    //list<Triangle*> map;



};

#endif // POLY_H
