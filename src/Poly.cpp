#include "Poly.h"


#define min(a,b)            (((a)<(b))?(a):(b))
#define max(a,b)            (((a)>(b))?(a):(b))

template <class C> void FreeClear( C & cntr ) {
    for ( typename C::iterator it = cntr.begin();
              it != cntr.end(); ++it ) {
        delete * it;
    }
    cntr.clear();
}
Poly::Poly()
{
  scaleFactor=1;
  pixels=nullptr;
  width=0;
  height=0;

}

Poly::~Poly()
{

FreeClear(polyline);
FreeClear(triangles);


    if(pixels)
    {
    UnloadImageColors(pixels);
    }

}

void Poly::reset()
{
FreeClear(polyline);
FreeClear(triangles);

}


void Poly::loadImage(Image rasterImage)
{

pixels = LoadImageColors(rasterImage);
width =rasterImage.width;
height=rasterImage.height;

}

///**************************************************************************************
float Poly::TriangulateArea(const Vector2* contour,int n)
{
  float A=0.0f;

  for(int p=n-1,q=0; q<n; p=q++)
  {
    A+= contour[p].x*contour[q].y - contour[q].x*contour[p].y;
  }
  return A*0.5f;
}

bool Poly::InsideTriangle(float Ax, float Ay,
                      float Bx, float By,
                      float Cx, float Cy,
                      float Px, float Py)
{
  float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
  float cCROSSap, bCROSScp, aCROSSbp;

  ax = Cx - Bx;  ay = Cy - By;
  bx = Ax - Cx;  by = Ay - Cy;
  cx = Bx - Ax;  cy = By - Ay;
  apx= Px - Ax;  apy= Py - Ay;
  bpx= Px - Bx;  bpy= Py - By;
  cpx= Px - Cx;  cpy= Py - Cy;

  aCROSSbp = ax*bpy - ay*bpx;
  cCROSSap = cx*apy - cy*apx;
  bCROSScp = bx*cpy - by*cpx;

  return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}


bool Poly::Snip(const  Vector2* contour,int u,int v,int w,int n,int *V)
{
  int p;
  float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

  Ax = contour[V[u]].x;
  Ay = contour[V[u]].y;

  Bx = contour[V[v]].x;
  By = contour[V[v]].y;

  Cx = contour[V[w]].x;
  Cy = contour[V[w]].y;

  if ( EPSILON > (((Bx-Ax)*(Cy-Ay)) - ((By-Ay)*(Cx-Ax))) ) return false;

  for (p=0;p<n;p++)
  {
    if( (p == u) || (p == v) || (p == w) ) continue;
    Px = contour[V[p]].x;
    Py = contour[V[p]].y;
    if (InsideTriangle(Ax,Ay,Bx,By,Cx,Cy,Px,Py)) return false;
  }

  return true;
}
int Poly::Triangulate(const Vector2* contour,int n,Vector2* result)
{
  /* allocate and initialize list of Vertices in polygon */


  if ( n < 3 )
  {
   TraceLog(LOG_DEBUG,"** Triangulate: ERROR - %d < 3!\n",n);
   return 0;
  }

  int *V = new int[n];

int total=0;


  /* we want a counter-clockwise polygon in V */

  if ( 0.0f < TriangulateArea(contour,n) )
    for (int v=0; v<n; v++) V[v] = v;
  else
    for(int v=0; v<n; v++) V[v] = (n-1)-v;

  int nv = n;

  /*  remove nv-2 Vertices, creating 1 triangle every time */
  int count = 2*nv;   /* error detection */

  for(int m=0, v=nv-1; nv>2; )
  {
    /* if we loop, it is probably a non-simple polygon */
    if (0 >= (count--))
    {
      TraceLog(LOG_DEBUG,"** Triangulate: ERROR - probable bad polygon!\n");
      return 0;
    }

    /* three consecutive vertices in current polygon, <u,v,w> */
    int u = v  ; if (nv <= u) u = 0;     /* previous */
    v = u+1; if (nv <= v) v = 0;     /* new v    */
    int w = v+1; if (nv <= w) w = 0;     /* next     */

    if ( Snip(contour,u,v,w,nv,V) )
    {
      int a,b,c,s,t;

      /* true names of the vertices */
      a = V[u]; b = V[v]; c = V[w];

      /* output Triangle */
      result[total++]=contour[a] ;
      result[total++]=contour[b] ;
      result[total++]=contour[c] ;

      m++;

      /* remove v from remaining polygon */
      for(s=v, t=v+1; t<nv; s++, t++)
      {
       V[s] = V[t];
       nv--;
      }

      /* resest error detection counter */
      count = 2*nv;
    }
  }



  delete V;

  return total;
}


int Poly::TriangulateArray(const Vector2* contour,int n,Tri* result)
{
  /* allocate and initialize list of Vertices in polygon */


  if ( n < 3 )
  {
   TraceLog(LOG_DEBUG,"** Triangulate: ERROR - %d < 3!\n",n);
   return 0;
  }

  int *V = new int[n];

int total=0;


  /* we want a counter-clockwise polygon in V */

  if ( 0.0f < TriangulateArea(contour,n) )
    for (int v=0; v<n; v++) V[v] = v;
  else
    for(int v=0; v<n; v++) V[v] = (n-1)-v;

  int nv = n;

  /*  remove nv-2 Vertices, creating 1 triangle every time */
  int count = 2*nv;   /* error detection */

  for(int m=0, v=nv-1; nv>2; )
  {
    /* if we loop, it is probably a non-simple polygon */
    if (0 >= (count--))
    {
      TraceLog(LOG_DEBUG,"** Triangulate: ERROR - probable bad polygon!\n");
      return 0;
    }

    /* three consecutive vertices in current polygon, <u,v,w> */
    int u = v  ; if (nv <= u) u = 0;     /* previous */
    v = u+1; if (nv <= v) v = 0;     /* new v    */
    int w = v+1; if (nv <= w) w = 0;     /* next     */

    if ( Snip(contour,u,v,w,nv,V) )
    {
      int a,b,c,s,t;

      /* true names of the vertices */
      a = V[u]; b = V[v]; c = V[w];

      /* output Triangle */
      result[total].face[0]=contour[a] ;
      result[total].face[1]=contour[b] ;
      result[total].face[2]=contour[c] ;
      total++;

      m++;

      /* remove v from remaining polygon */
      for(s=v,t=v+1;t<nv;s++,t++) V[s] = V[t]; nv--;

      /* resest error detection counter */
      count = 2*nv;
    }
  }



  delete V;

  return total;
}





unsigned char Poly::getAlphaPixel(int x, int y)
{
   if (x<0) x=0;
   if (y<0) y=0;
   if (x>width)  x=width;
   if (y>height) y=height;

    Color c=pixels[y*width+x];
    return c.a;
}



unsigned char Poly::getAlphaByPos(const Vector2& pos)
{

    Color c=pixels[(int)pos.y*width+(int)pos.x];
    return c.a;
}
int Poly::getIndexFromPos(unsigned int x, unsigned int y)
{ return y*width+x; }

unsigned char* Poly::getImageColors(Image image)
{


       unsigned char *pixels = (unsigned char *)RL_MALLOC(image.width*image.height*4);
        int index=0;
        for (int i = 0, k = 0; i < image.width*image.height; i++)
        {
                    pixels[index] = ((unsigned char *)image.data)[k + 0];index++;
                    pixels[index] = ((unsigned char *)image.data)[k + 1];index++;
                    pixels[index] = ((unsigned char *)image.data)[k + 2];index++;
                    pixels[index] = ((unsigned char *)image.data)[k + 3];index++;

                    k += 4;

        }


    return pixels;
}

Vector2 Poly::findFirstNoneTransparentPixel(const Rectangle& rect, float threshold)
{
	bool found = false;
    Vector2 i;
    for(i.y = rect.y; i.y < rect.y+rect.height; i.y++)
    {
        if(found)break;
        for(i.x = rect.x; i.x < rect.x+rect.width; i.x++)
        {
            auto alpha = getAlphaByPos(i);
            if(alpha>threshold)
            {
                found = true;
                break;
            }
        }
    }
    return i;
}

unsigned int Poly::getSquareValue(unsigned int x, unsigned int y, const Rectangle& rect, float threshold)
{
    /*
     checking the 2x2 pixel grid, assigning these values to each pixel, if not transparent
     +---+---+
     | 1 | 2 |
     +---+---+
     | 4 | 8 | <- current pixel (curx,cury)
     +---+---+
     */
    unsigned int sv = 0;
    //NOTE: due to the way we pick points from texture, rect needs to be smaller, otherwise it goes outside 1 pixel
    Rectangle fixedRect = {rect.x,rect.y, rect.width-2,rect.height-2};

    Vector2 tl = {x-1.0f, y-1.0f};
    sv += (CheckCollisionPointRec(tl,fixedRect) && getAlphaByPos(tl) > threshold)? 1 : 0;
    Vector2 tr = {x-0.0f, y-1.0f};
    sv += (CheckCollisionPointRec(tr,fixedRect) && getAlphaByPos(tr) > threshold)? 2 : 0;
    Vector2 bl = {x-1.0f, y-0.0f};
    sv += (CheckCollisionPointRec(bl,fixedRect)&& getAlphaByPos(bl) > threshold)? 4 : 0;
    Vector2 br = {x-0.0f, y-0.0f};
    sv += (CheckCollisionPointRec(br,fixedRect) && getAlphaByPos(br) > threshold)? 8 : 0;
    if (sv != 0 && sv != 15);
    TraceLog(LOG_DEBUG,"square value should not be 0, or 15");
    return sv;
}


std::vector<Vector2> Poly::marchSquare(const Rectangle& rect, const Vector2& start, float threshold)
{

    int stepx = 0;
    int stepy = 0;
    int prevx = 0;
    int prevy = 0;
    int startx = (int)start.x;
    int starty = (int)start.y;
    int curx = startx;
    int cury = starty;
    unsigned int count = 0;
    std::vector<int> case9s;
    std::vector<int> case6s;
    int i;
    std::vector<int>::iterator it;
    std::vector<Vector2> _points;
    do{
        int sv = getSquareValue(curx, cury, rect, threshold);
        switch(sv){

            case 1:
            case 5:
            case 13:
                /* going UP with these cases:
                 1          5           13
                 +---+---+  +---+---+  +---+---+
                 | 1 |   |  | 1 |   |  | 1 |   |
                 +---+---+  +---+---+  +---+---+
                 |   |   |  | 4 |   |  | 4 | 8 |
                 +---+---+  +---+---+  +---+---+
                 */
                stepx = 0;
                stepy = -1;
                break;


            case 8:
            case 10:
            case 11:
                /* going DOWN with these cases:
                 8          10          11
                 +---+---+  +---+---+   +---+---+
                 |   |   |  |   | 2 |   | 1 | 2 |
                 +---+---+  +---+---+   +---+---+
                 |   | 8 |  |   | 8 |   |   | 8 |
                 +---+---+  +---+---+  	+---+---+
                 */
                stepx = 0;
                stepy = 1;
                break;


            case 4:
            case 12:
            case 14:
                /* going LEFT with these cases:
                 4          12          14
                 +---+---+  +---+---+   +---+---+
                 |   |   |  |   |   |   |   | 2 |
                 +---+---+  +---+---+   +---+---+
                 | 4 |   |  | 4 | 8 |   | 4 | 8 |
                 +---+---+  +---+---+  	+---+---+
                 */
                stepx = -1;
                stepy = 0;
                break;


            case 2 :
            case 3 :
            case 7 :
                /* going RIGHT with these cases:
                 2          3           7
                 +---+---+  +---+---+   +---+---+
                 |   | 2 |  | 1 | 2 |   | 1 | 2 |
                 +---+---+  +---+---+   +---+---+
                 |   |   |  |   |   |   | 4 |   |
                 +---+---+  +---+---+  	+---+---+
                 */
                stepx=1;
                stepy=0;
                break;
            case 9 :
                /*
                 +---+---+
                 | 1 |   |
                 +---+---+
                 |   | 8 |
                 +---+---+
                 this should normally go UP, but if we already been here, we go down
                */
                //find index from xy;
                i = getIndexFromPos(curx, cury);
                it = find (case9s.begin(), case9s.end(), i);
                if (it != case9s.end())
                {
                    //found, so we go down, and delete from case9s;
                    stepx = 0;
                    stepy = 1;
                    case9s.erase(it);
                }
                else
                {
                    //not found, we go up, and add to case9s;
                    stepx = 0;
                    stepy = -1;
                    case9s.push_back(i);
                }
                break;
            case 6 :
                /*
                 6
                 +---+---+
                 |   | 2 |
                 +---+---+
                 | 4 |   |
                 +---+---+
                 this normally go RIGHT, but if its coming from UP, it should go LEFT
                 */
                i = getIndexFromPos(curx, cury);
                it = find (case6s.begin(), case6s.end(), i);
                if (it != case6s.end())
                {
                    //found, so we go down, and delete from case9s;
                    stepx = -1;
                    stepy = 0;
                    case6s.erase(it);
                }
                else{
                    //not found, we go up, and add to case9s;
                    stepx = 1;
                    stepy = 0;
                    case6s.push_back(i);
                }
                break;
            default:
                TraceLog(LOG_DEBUG,"this shouldn't happen.");
        }
        //little optimization
        // if previous direction is same as current direction,
        // then we should modify the last vec to current
        curx += stepx;
        cury += stepy;
        if(stepx == prevx && stepy == prevy)
        {

            _points.back().x = (float)(curx+rect.x) / scaleFactor;
            _points.back().y = (float)(cury+rect.y) / scaleFactor;
        }
        else
        {
             Vector2 v ;
             v.x=(float)(curx + rect.x) / scaleFactor;
             v.y=(float)(cury + rect.y) / scaleFactor;
            _points.push_back(v);
            //TraceLog(LOG_INFO,"%f %f",v.x,v.y);
        }

        count++;
        prevx = stepx;
        prevy = stepy;


        const auto totalPixel = width * height;
        if (count <= totalPixel)
        TraceLog(LOG_DEBUG, "oh no, marching square cannot find starting position");

    } while(curx != startx || cury != starty);
    return _points;
}

float Poly::perpendicularDistance(const Vector2& i, const Vector2& start, const Vector2& end)
{
    float res;
    float slope;
    float intercept;

    if(start.x == end.x)
    {
        res = fabsf(i.x- end.x);
    }
    else if (start.y == end.y)
    {
        res = fabsf(i.y - end.y);
    }
    else{
        slope = (end.y - start.y) / (end.x - start.x);
        intercept = start.y - (slope*start.x);
        res = fabsf(slope * i.x - i.y + intercept) / sqrtf(powf(slope, 2)+1);
    }
    return res;
}
std::vector<Vector2> Poly::rdp(const std::vector<Vector2>& v, float optimization)
{
    if(v.size() < 3)
        return v;

    int index = -1;
    float dist = 0;
    //not looping first and last point
    for(size_t i = 1, size = v.size(); i < size-1; ++i)
    {
        float cdist = perpendicularDistance(v[i], v.front(), v.back());
        if(cdist > dist)
        {
            dist = cdist;
            index = static_cast<int>(i);
        }
    }
    if (dist>optimization)
    {
        std::vector<Vector2>::const_iterator begin = v.begin();
        std::vector<Vector2>::const_iterator end   = v.end();
        std::vector<Vector2> l1(begin, begin+index+1);
        std::vector<Vector2> l2(begin+index, end);

        std::vector<Vector2> r1 = rdp(l1, optimization);
        std::vector<Vector2> r2 = rdp(l2, optimization);

        r1.insert(r1.end(), r2.begin()+1, r2.end());
        return r1;
    }
    else {
        std::vector<Vector2> ret;
        ret.push_back(v.front());
        ret.push_back(v.back());
        return ret;
    }
}
std::vector<Vector2> Poly::trace(const Rectangle& rect, float threshold)
{
    Vector2 first = findFirstNoneTransparentPixel(rect, threshold);
    return marchSquare(rect, first, threshold);
}

std::vector<Vector2> Poly::process(const Rectangle& rect, float threshold,float epsilon)
{
   std::vector<Vector2> points = trace(rect,threshold);
   points = reduce(points,rect,epsilon);
   return points;
}

std::vector<Vector2> Poly::reduce(const std::vector<Vector2>& points, const Rectangle& rect, float epsilon)
{


    auto size = points.size();
    // if there are less than 3 points, then we have nothing
    if(size<3)
    {
        TraceLog(LOG_DEBUG,"AUTOPOLYGON: cannot reduce points ");
        return std::vector<Vector2>();
    }
    // if there are less than 9 points (but more than 3), then we don't need to reduce it
    else if (size < 9)
    {
        TraceLog(LOG_DEBUG,"AUTOPOLYGON: cannot reduce points ");
        return points;
    }
    float maxEp = min(rect.width, rect.height);
    float ep = Clamp(epsilon, 0.0, maxEp/scaleFactor/2);
    std::vector<Vector2> result = rdp(points, ep);

    auto last = result.back();
    if (last.y > result.front().y && Vector2Distance(last,result.front()) < ep * 0.5f)
    {
        result.front().y = last.y;
        result.pop_back();
    }
    return result;
}
