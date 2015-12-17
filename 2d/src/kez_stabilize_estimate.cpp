/* Scale, Rotation, Translation, constrained on uncovered area, and scaling */
/* Trying different weighting. */

#include <gsl/gsl_multifit_nlin.h>

#include <limits>
#include <highgui.h>

#include "sift_feature.h"
#include "sift_particle_video.h"
#include "sift_stabilize.h"
#include "cv.h"

using namespace SIFT;
using namespace ParticleVideo;
using std::cout;
using std::endl;

struct ParamT {
  ParamT() : begin_frame(0), end_frame(0) {}
  ParamT(
      const vector< vector<ParticleIdx> > *filtered_particles, 
      const SIFTParticleMaps *pms,
      vector<SimilarityTransformMatrix> *tList,
      const unsigned int begin_frame,
      const unsigned int end_frame) :
    filtered_particles(filtered_particles),
    pms(pms),
    tList(tList),
    begin_frame(begin_frame),
    end_frame(end_frame) {}

  const vector< vector<ParticleIdx> > *filtered_particles;
  const SIFTParticleMaps *pms;
  vector<SimilarityTransformMatrix> *tList; 
  const unsigned int begin_frame;
  const unsigned int end_frame;
};

std::map<std::pair<int, ParticleIdx>, double> WeightMap;
static const unsigned int MIN_FILTERED = 5;
static const unsigned int FILTERED = 30;
static const double DELTA_REG = 1e-6;
static const double SCALE_TOR = 1e-1;
static const double SIGMA_SMOOTH = 10;

static const double LAMBDA_SMOOTH = 0;
//static const double LAMBDA1 = 1;
//static const double LAMBDA_SCALE_REGUR= LAMBDA1 * 1e3;
//static const double LAMBDA_UNCOVERED = LAMBDA1 * 1e3;
static double LAMBDA_SCALE_REGUR;
static double LAMBDA_UNCOVERED;

static const double MARGIN_RATIO = 0;
static int FRAME_WIDTH;
static int FRAME_HEIGHT;

static double uncovered_cost(
    const SimilarityTransformMatrix &m, 
    const int w,
    const int h) {
  double v = 0;
  CvPoint2D32f p[4] = {
    { 0, 0 },
    { FRAME_WIDTH-1, 0},
    { 0, FRAME_HEIGHT-1},
    { FRAME_WIDTH-1, FRAME_HEIGHT-1}};

  const double s = sqrt(m.a*m.a+m.b*m.b);
  const double cos = m.a/s;
  const double sin = m.b/s;
  const double dx  = m.dx/s;
  const double dy  = m.dy/s;

  for (int k = 0; k < 4; ++k) {
    const double ox = p[k].x;
    const double oy = p[k].y;
    const double x = (cos*ox + sin*oy)/s - (cos*dx+sin*dy);
    const double y = (-sin*ox + cos*oy)/s - (-sin*dx+cos*dy);
    const double mid_x = (FRAME_WIDTH-1)/2.;
    const double mid_y = (FRAME_HEIGHT-1)/2.;
    const double w = (FRAME_WIDTH)/2.;
    const double h = (FRAME_HEIGHT)/2.;
    const double ex = fabs(x-mid_x) - w < 0 ? 0 : fabs(x-mid_x) - w;
    const double ey = fabs(y-mid_y) - h < 0 ? 0 : fabs(y-mid_y) - h;
    v +=
      FRAME_HEIGHT*(sqrt(ex*ex+DELTA_REG*DELTA_REG) - DELTA_REG) +
      FRAME_WIDTH*(sqrt(ey*ey+DELTA_REG*DELTA_REG) - DELTA_REG);
  }
  return v/FRAME_WIDTH/FRAME_HEIGHT;
}

int s_cost_f (const gsl_vector * x, void * params, gsl_vector * f)  {
  ParamT *p = (ParamT *) params;

  const vector<vector<ParticleIdx> > &filtered_particles =
    *(p->filtered_particles);

  const SIFTParticleMaps &pms =
    *(p->pms);

  const unsigned int begin_frame = p->begin_frame;
  const unsigned int end_frame   = p->end_frame;
  const unsigned int window_width = end_frame-begin_frame+1;

  vector<SimilarityTransformMatrix> &tList = *(p->tList);

  for (unsigned int i = 0; i < window_width; ++i) {
    SimilarityTransformMatrix &t = tList[begin_frame+i];

    const double theta =     gsl_vector_get(x, 4*i+2);
    const double scale_var = gsl_vector_get(x, 4*i+3);
    const double scale =
      sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

    t.sv = scale_var;

    t.dx = scale * gsl_vector_get(x, 4*i);
    t.dy = scale * gsl_vector_get(x, 4*i+1);

    t.a  = scale*cos(theta);
    t.b  = scale*sin(theta);
  }

  gsl_vector_set_all(f, 0);

  for (int i = -1; i <= (signed) window_width; ++i) {
    const int f0 = begin_frame+i-1;
    const int f1 = begin_frame+i;
    const int f2 = begin_frame+i+1;

    if (f0 >= 0 && f2 < (signed)pms.size()) {
      const double dx0 = tList[f0].dx;
      const double dx1 = tList[f1].dx;
      const double dx2 = tList[f2].dx;

      const double dy0 = tList[f0].dy;
      const double dy1 = tList[f1].dy;
      const double dy2 = tList[f2].dy;

      const double a0 = tList[f0].a;
      const double a1 = tList[f1].a;
      const double a2 = tList[f2].a;

      const double b0 = tList[f0].b;
      const double b1 = tList[f1].b;
      const double b2 = tList[f2].b;

      const double sv1 = tList[f1].sv;
      const double s1 = sqrt(sv1*sv1+SimilarityTransformMatrix::DELTA_SCALE);

      if ( filtered_particles[f1].size() >= MIN_FILTERED) {
        for (unsigned int j = 0; j < filtered_particles[f1].size(); ++j) {
          const ParticleIdx particle_num = filtered_particles[f1][j];

          const CvPoint2D32f p0 = pms.getPosition(f0, particle_num);
          const CvPoint2D32f p1 = pms.getPosition(f1, particle_num);
          const CvPoint2D32f p2 = pms.getPosition(f2, particle_num);

          const double ax =
            (    a0*p0.x - b0*p0.y + dx0)
            -2*( a1*p1.x - b1*p1.y + dx1)
            + (  a2*p2.x - b2*p2.y + dx2);

          const double ay =
            (    b0*p0.x + a0*p0.y + dy0)
            -2*( b1*p1.x + a1*p1.y + dy1)
            + (  b2*p2.x + a2*p2.y + dy2);

          int tidx =  FILTERED*(i+1) + j;

          double weight = sqrt(WeightMap[std::make_pair(f1, particle_num)]);
          gsl_vector_set( f, 2*tidx+0, weight*ax/s1 );
          gsl_vector_set( f, 2*tidx+1, weight*ay/s1 );
        }
      } else {
        const CvPoint2D32f pt[4] = {
          {0, 0},
          {FRAME_WIDTH, 0},
          {0, FRAME_HEIGHT},
          {FRAME_WIDTH, FRAME_HEIGHT} };
        for (unsigned int k = 0; k < 4; ++k) {
          const CvPoint2D32f p0 = pt[k];
          const CvPoint2D32f p1 = pt[k];
          const CvPoint2D32f p2 = pt[k];

          const double ax =
            (    a0*p0.x - b0*p0.y + dx0)
            -2*( a1*p1.x - b1*p1.y + dx1)
            + (  a2*p2.x - b2*p2.y + dx2);

          const double ay =
            (    b0*p0.x + a0*p0.y + dy0)
            -2*( b1*p1.x + a1*p1.y + dy1)
            + (  b2*p2.x + a2*p2.y + dy2);

          int tidx =  FILTERED*(i+1) + k;

          double weight = sqrt(1./4);
          gsl_vector_set( f, 2*tidx, weight*ax/s1 );
          gsl_vector_set( f, 2*tidx+1, weight*ay/s1 );
        }
      }
    }
  }

  {
    int tidx = 2*FILTERED*(window_width+1);

    for (unsigned int i = 0; i < window_width; ++i) {
      const SimilarityTransformMatrix &m =
        tList[begin_frame+i];

      const double scale = sqrt(m.a*m.a+m.b*m.b);
      if (scale > 1 ) 
        gsl_vector_set( f, tidx+i,
            LAMBDA_SCALE_REGUR*(scale-1)*(scale-1));

      gsl_vector_set( f, tidx+window_width+i,
          LAMBDA_UNCOVERED*uncovered_cost(m, FRAME_WIDTH, FRAME_HEIGHT));
    }
  }

  {
    double sum1 = 0;
    double sum2 = 0;
    double sum4 = 0;
    for (unsigned int i = 0; i < 2*FILTERED*(window_width+1); ++i)
      sum1 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
    for (unsigned int i = 2*FILTERED*(window_width+1); i < 2*FILTERED*(window_width+1)+window_width; ++i)
      sum2 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);
    for (unsigned int i = 2*FILTERED*(window_width+1)+window_width;
        i < 2*FILTERED*(window_width+1)+2*window_width; ++i)
      sum4 += gsl_vector_get( f, i ) * gsl_vector_get(f, i);

    std::cout << "Roughnesss: " << sum1 << std::endl;
    std::cout << "Scaling: " << sum2 << std::endl;
    std::cout << "Uncovered: " << sum4 << std::endl;
    std::cout << "Cost: " << sum1 + sum2 + sum4 << std::endl << std::endl;
  }
  return GSL_SUCCESS;
}

int s_cost_df (const gsl_vector * x, void *params, gsl_matrix * J) {
  ParamT *p = (ParamT *) params;

  const vector<vector<ParticleIdx> > &filtered_particles =
    *(p->filtered_particles);

  const SIFTParticleMaps &pms =
    *(p->pms);

  const unsigned int begin_frame = p->begin_frame;
  const unsigned int end_frame   = p->end_frame;

  const unsigned int window_width = end_frame - begin_frame+1;

  vector<SimilarityTransformMatrix> &tList = *(p->tList);

  for (unsigned int i = 0; i < window_width; ++i) {
    SimilarityTransformMatrix &t = tList[begin_frame+i];

    const double theta = gsl_vector_get(x, 4*i+2);
    const double scale_var = gsl_vector_get(x, 4*i+3);
    const double scale =
      sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

    t.sv = scale_var;

    t.dx = scale * gsl_vector_get(x, 4*i);
    t.dy = scale * gsl_vector_get(x, 4*i+1);

    t.a  = scale*cos(theta);
    t.b  = scale*sin(theta);
  }

  gsl_matrix_set_all(J, 0);

  for (int i = -1; i <= (signed)window_width; ++i) {
    const int i0 = i-1;
    const int i1 = i;
    const int i2 = i+1;

    const int f0 = begin_frame+i-1;
    const int f1 = begin_frame+i;
    const int f2 = begin_frame+i+1;

    if (f0 >= 0 && f2 < (signed) pms.size()) {
      const double dx0 = tList[f0].dx;
      const double dx2 = tList[f2].dx;

      const double dy0 = tList[f0].dy;
      const double dy2 = tList[f2].dy;

      const double a0 = tList[f0].a;
      const double a1 = tList[f1].a;
      const double a2 = tList[f2].a;

      const double b0 = tList[f0].b;
      const double b1 = tList[f1].b;
      const double b2 = tList[f2].b;

      const int ix0 = 4*i0;
      const int ix1 = 4*i1;
      const int ix2 = 4*i2;

      const int iy0 = 4*i0+1;
      const int iy1 = 4*i1+1;
      const int iy2 = 4*i2+1;

      const int it0 = 4*i0+2;
      const int it1 = 4*i1+2;
      const int it2 = 4*i2+2;

      const int is0 = 4*i0+3;
      const int is1 = 4*i1+3;
      const int is2 = 4*i2+3;

      const double sv0 = tList[f0].sv;
      const double sv1 = tList[f1].sv;
      const double sv2 = tList[f2].sv;

      const double s0 = sqrt(sv0*sv0+SimilarityTransformMatrix::DELTA_SCALE);
      const double s1 = sqrt(sv1*sv1+SimilarityTransformMatrix::DELTA_SCALE);
      const double s2 = sqrt(sv2*sv2+SimilarityTransformMatrix::DELTA_SCALE);

      if ( filtered_particles[f1].size() >= MIN_FILTERED) {
        for (unsigned int j = 0; j < filtered_particles[f1].size(); ++j) {
          const ParticleIdx particle_num = filtered_particles[f1][j];

          const CvPoint2D32f p0 = pms.getPosition(f0, particle_num);
          const CvPoint2D32f p1 = pms.getPosition(f1, particle_num);
          const CvPoint2D32f p2 = pms.getPosition(f2, particle_num);

          unsigned int fidx = FILTERED*(i+1)+j;

          double weight = sqrt(WeightMap[std::make_pair(f1, particle_num)]);

          if (i0 >= 0) {
            gsl_matrix_set(J, 2*fidx,   ix0, s0/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, ix0, 0);

            gsl_matrix_set(J, 2*fidx,   iy0, 0);
            gsl_matrix_set(J, 2*fidx+1, iy0, s0/s1*weight);

            gsl_matrix_set(J, 2*fidx,   it0, (-b0*p0.x-a0*p0.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it0, (a0*p0.x-b0*p0.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is0, (a0*p0.x-b0*p0.y+dx0)/s0*sv0/s0/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is0, (b0*p0.x+a0*p0.y+dy0)/s0*sv0/s0/s1*weight);
          }


          if (i2 < (signed) window_width) {
            gsl_matrix_set(J, 2*fidx,   ix2, s2/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, ix2, 0);

            gsl_matrix_set(J, 2*fidx,   iy2, 0);
            gsl_matrix_set(J, 2*fidx+1, iy2, s2/s1*weight);

            gsl_matrix_set(J, 2*fidx,   it2, (-b2*p2.x-a2*p2.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it2, (a2*p2.x-b2*p2.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is2, (a2*p2.x-b2*p2.y+dx2)/s2*sv2/s2/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is2, (b2*p2.x+a2*p2.y+dy2)/s2*sv2/s2/s1*weight);
          }

          if (i1 >= 0 && i1 < (signed) window_width) {
            gsl_matrix_set(J, 2*fidx,   ix1, -2*weight);
            gsl_matrix_set(J, 2*fidx+1, ix1, 0);

            gsl_matrix_set(J, 2*fidx,   iy1, 0);
            gsl_matrix_set(J, 2*fidx+1, iy1, -2*weight);

            gsl_matrix_set(J, 2*fidx,   it1, -2*(-b1*p1.x-a1*p1.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it1, -2*(a1*p1.x-b1*p1.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is1, 
                -(a0*p0.x-b0*p0.y+dx0 + a2*p2.x-b2*p2.y+dx2)*sv1/s1/s1/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is1, 
                -(b0*p0.x+a0*p0.y+dy0 + b2*p2.x+a2*p2.y+dy2)*sv1/s1/s1/s1*weight);
          }
        }
      } else {
        const CvPoint2D32f pt[4] = {
          {0, 0},
          {FRAME_WIDTH, 0},
          {0, FRAME_HEIGHT},
          {FRAME_WIDTH, FRAME_HEIGHT} };
        for (unsigned int k = 0; k < 4; ++k) {
          const CvPoint2D32f p0 = pt[k];
          const CvPoint2D32f p1 = pt[k];
          const CvPoint2D32f p2 = pt[k];

          unsigned int fidx = FILTERED*(i+1)+k;

          double weight = sqrt(1./4);

          if (i0 >= 0) {
            gsl_matrix_set(J, 2*fidx,   ix0, s0/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, ix0, 0);

            gsl_matrix_set(J, 2*fidx,   iy0, 0);
            gsl_matrix_set(J, 2*fidx+1, iy0, s0/s1*weight);

            gsl_matrix_set(J, 2*fidx,   it0, (-b0*p0.x-a0*p0.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it0, (a0*p0.x-b0*p0.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is0, (a0*p0.x-b0*p0.y+dx0)/s0*sv0/s0/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is0, (b0*p0.x+a0*p0.y+dy0)/s0*sv0/s0/s1*weight);
          }


          if (i2 < (signed) window_width) {
            gsl_matrix_set(J, 2*fidx,   ix2, s2/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, ix2, 0);

            gsl_matrix_set(J, 2*fidx,   iy2, 0);
            gsl_matrix_set(J, 2*fidx+1, iy2, s2/s1*weight);

            gsl_matrix_set(J, 2*fidx,   it2, (-b2*p2.x-a2*p2.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it2, (a2*p2.x-b2*p2.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is2, (a2*p2.x-b2*p2.y+dx2)/s2*sv2/s2/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is2, (b2*p2.x+a2*p2.y+dy2)/s2*sv2/s2/s1*weight);
          }

          if (i1 >= 0 && i1 < (signed) window_width) {
            gsl_matrix_set(J, 2*fidx,   ix1, -2*weight);
            gsl_matrix_set(J, 2*fidx+1, ix1, 0);

            gsl_matrix_set(J, 2*fidx,   iy1, 0);
            gsl_matrix_set(J, 2*fidx+1, iy1, -2*weight);

            gsl_matrix_set(J, 2*fidx,   it1, -2*(-b1*p1.x-a1*p1.y)/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, it1, -2*(a1*p1.x-b1*p1.y)/s1*weight);

            gsl_matrix_set(J, 2*fidx,   is1, 
                -(a0*p0.x-b0*p0.y+dx0 + a2*p2.x-b2*p2.y+dx2)*sv1/s1/s1/s1*weight);
            gsl_matrix_set(J, 2*fidx+1, is1, 
                -(b0*p0.x+a0*p0.y+dy0 + b2*p2.x+a2*p2.y+dy2)*sv1/s1/s1/s1*weight);
          }
        }
      }
    }
  }

  {
    int tidx = 2*FILTERED*(window_width+1);
    for (unsigned int i = 0; i < window_width; ++i) {
      const int jx = 4*i;
      const int jy = 4*i+1;
      const int jt = 4*i+2;
      const int js = 4*i+3;

      const SimilarityTransformMatrix &m = tList[begin_frame+i];
      const double sv = m.sv;
      const double scale =
        sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);

      if (scale > 1)
        gsl_matrix_set( J, tidx+i, js, LAMBDA_SCALE_REGUR * 2 * (scale-1) * sv /scale); 

      {
        CvPoint2D32f p[4] = {
          { 0, 0 },
          { FRAME_WIDTH-1, 0},
          { 0, FRAME_HEIGHT-1},
          { FRAME_WIDTH-1, FRAME_HEIGHT-1}};

        {
          double sum_x = 0;
          double sum_y = 0;
          double sum_t = 0;
          double sum_s = 0;
          const double s = sqrt(m.a*m.a+m.b*m.b);
          const double cos = m.a/s;
          const double sin = m.b/s;
          const double dx  = m.dx/s;
          const double dy  = m.dy/s;

          const double mid_x = (FRAME_WIDTH-1)/2.;
          const double mid_y = (FRAME_HEIGHT-1)/2.;
          const double w = (FRAME_WIDTH)/2.;
          const double h = (FRAME_HEIGHT)/2.;

          for (int k = 0; k < 4; ++k) {
            const double ox = p[k].x;
            const double oy = p[k].y;
            const double x = (cos*ox + sin*oy)/s - (cos*dx+sin*dy);
            const double y = (-sin*ox + cos*oy)/s - (-sin*dx+cos*dy);

            const double ex = fabs(x-mid_x) - w < 0 ? 0 : fabs(x-mid_x) - w;
            const double ey = fabs(y-mid_y) - h < 0 ? 0 : fabs(y-mid_y) - h;

            const double sign_x = x > mid_x ? 1 : -1;
            const double sign_y = y > mid_y ? 1 : -1;

            const double cx = sqrt(ex*ex+DELTA_REG*DELTA_REG) * FRAME_WIDTH;
            const double cy = sqrt(ey*ey+DELTA_REG*DELTA_REG) * FRAME_HEIGHT;

            sum_x += ex*(sign_x*(-cos))/cx + ey*sign_y*sin/cy;
            sum_y += ex*(sign_x*(-sin))/cx + ey*sign_y*(-cos)/cy;
            sum_t +=
              ex*sign_x*((-sin*ox+cos*oy)/s-(-sin*dx+cos*dy))/cx +
              ey*sign_y*((-cos*ox-sin*oy)/s-(-cos*dx-sin*dy))/cy;
            sum_s += 
              ex*sign_x*(-(cos*ox+sin*oy)/s/s*sv/s)/cx +
              ey*sign_y*(-(-sin*ox+cos*oy)/s/s*sv/s)/cy;
          }

          gsl_matrix_set( J, tidx+window_width+i, jx, LAMBDA_UNCOVERED*sum_x);
          gsl_matrix_set( J, tidx+window_width+i, jy, LAMBDA_UNCOVERED*sum_y);
          gsl_matrix_set( J, tidx+window_width+i, jt, LAMBDA_UNCOVERED*sum_t);
          gsl_matrix_set( J, tidx+window_width+i, js, LAMBDA_UNCOVERED*sum_s);
        }
      }
    }
  }
  return GSL_SUCCESS;
}

int s_cost_fdf(const gsl_vector * x, void *data, gsl_vector * f, gsl_matrix * J) {
  s_cost_f(x, data, f);
  s_cost_df(x, data, J);
  return GSL_SUCCESS;
}


double optimizeSimilarityTransfomation(
    vector<SimilarityTransformMatrix> &tList,
    vector< vector<ParticleIdx> > &filtered_particles,
    const SIFTParticleMaps &pms,
    unsigned int begin_frame,
    unsigned int end_frame) {

  unsigned int window_width = end_frame-begin_frame+1;

  ParamT param(&filtered_particles, &pms, &tList, begin_frame, end_frame);

  const int n =
    2 * FILTERED * (window_width+1) +   // roughness
    window_width +                  // scaling
    window_width;               // uncovered

  const int p = 4 * window_width;

  gsl_multifit_function_fdf cost_function;
  {
    cost_function.n		    = n;
    cost_function.f		    = &s_cost_f;
    cost_function.df	    = &s_cost_df;
    cost_function.fdf     = &s_cost_fdf;
    cost_function.p       = p;
    cost_function.params  = (void *)&param;
  }

  const gsl_multifit_fdfsolver_type *T =
    gsl_multifit_fdfsolver_lmsder;

  gsl_vector *x =
    gsl_vector_alloc( p );

  for (unsigned int i = 0; i < window_width; ++i) {
    SimilarityTransformMatrix &t = tList[begin_frame+i];
    const double scale = sqrt(t.a*t.a+t.b*t.b);

    gsl_vector_set(x, 4*i, t.dx / scale);
    gsl_vector_set(x, 4*i+1, t.dy / scale);
    gsl_vector_set(x, 4*i+2, atan2(t.b, t.a));
    gsl_vector_set(x, 4*i+3, t.sv);
  }

  gsl_multifit_fdfsolver *s =
    gsl_multifit_fdfsolver_alloc(T, n, p);

  gsl_multifit_fdfsolver_set(s, &cost_function, x);

  size_t iter = 0;
  int status;

  do {
    iter++;
    std::cout << "Iteration: #" << iter << std::endl;
    status = gsl_multifit_fdfsolver_iterate(s);
    if (status) break;

    status = gsl_multifit_test_delta(s->dx, s->x, window_width*1e-3,
        window_width*1e-3);

    if (status == GSL_SUCCESS) printf ("Minimum found at:\n"); 
  }
  while (status == GSL_CONTINUE && iter < 15); 
  std::cout << std::endl;

  for (unsigned int i = 0; i < window_width; ++i) {
    const double theta =
      gsl_vector_get( s->x, 4*i+2 );

    const double scale_var =
      gsl_vector_get( s->x, 4*i+3 );

    const double scale =
      sqrt(scale_var*scale_var+SimilarityTransformMatrix::DELTA_SCALE);

    tList[begin_frame+i].a = scale*cos(theta);
    tList[begin_frame+i].b = scale*sin(theta);

    tList[begin_frame+i].dx = scale*gsl_vector_get(s->x, 4*i);
    tList[begin_frame+i].dy = scale*gsl_vector_get(s->x, 4*i+1);

    tList[begin_frame+i].sv = scale_var;
  }

  double result = 0;
  for (unsigned int i = 0; i < 2*FILTERED*(window_width+1)+2*window_width; ++i)
    result += gsl_vector_get( s->f, i ) * gsl_vector_get(s->f, i);

  std::cout << std::endl;
  gsl_multifit_fdfsolver_free(s); // L: solver from GUN for the objective function
  gsl_vector_free(x);
  return result;
}

const int WINDOW_SIZE = 60;
const int WINDOW_STEP = WINDOW_SIZE/2;

void filtered_tracks(
    vector< vector<ParticleIdx> > &filtered_particles,
    const SIFTParticleMaps &pms) {
  // {{{
  for (int frame_num = 1; frame_num < (signed)pms.size()-1; ++frame_num) {
    vector< std::pair<double, ParticleIdx> > list; 
    const SIFTParticleMap &pm = pms[frame_num];
    std::vector<ParticleIdx> particles;
    for (
        SIFTParticleMap::const_iterator p = pm.begin();
        p != pm.end();
        ++p) {
      ParticleIdx particle_num = p->first;

      if (
          pms.exist(frame_num-1, particle_num) &&
          pms.exist(frame_num+1, particle_num)) {

        double cost = 0;
        double weight_sum = 0;
        {
          const CvPoint2D32f p0 = pms.getPosition(frame_num-1, particle_num);
          const CvPoint2D32f p1 = pms.getPosition(frame_num, particle_num);
          const CvPoint2D32f p2 = pms.getPosition(frame_num+1, particle_num);
          const double ax = p0.x-2*p1.x+p2.x;
          const double ay = p0.y-2*p1.y+p2.y;
          const double vx = p1.x-p2.x;
          const double vy = p1.y-p2.y;
          cost +=  (ax*ax+ay*ay);
          weight_sum += 1;
        }

        int length = 2;
        while (
            pms.exist(frame_num+length, particle_num) &&
            pms.exist(frame_num-length, particle_num)) {
          {
            const CvPoint2D32f p0 =
              pms.getPosition(frame_num+length-2, particle_num);
            const CvPoint2D32f p1 =
              pms.getPosition(frame_num+length-1, particle_num);
            const CvPoint2D32f p2 =
              pms.getPosition(frame_num+length, particle_num);

            const double ax = p0.x-2*p1.x+p2.x;
            const double ay = p0.y-2*p1.y+p2.y;
            const double vx = p1.x-p2.x;
            const double vy = p1.y-p2.y;
            cost +=  exp(-(length*length))*(ax*ax+ay*ay);
            weight_sum += exp(-(length*length));
          }

          {
            const CvPoint2D32f p0 =
              pms.getPosition(frame_num-length+2, particle_num);
            const CvPoint2D32f p1 =
              pms.getPosition(frame_num-length+1, particle_num);
            const CvPoint2D32f p2 =
              pms.getPosition(frame_num-length, particle_num);

            const double ax = p0.x-2*p1.x+p2.x;
            const double ay = p0.y-2*p1.y+p2.y;
            const double vx = p1.x-p2.x;
            const double vy = p1.y-p2.y;
            cost +=  exp(-(length*length))*(ax*ax+ay*ay);
            weight_sum += exp(-(length*length)/SIGMA_SMOOTH/SIGMA_SMOOTH);
          }
          ++length;
        }; 
        cost /= weight_sum;
        list.push_back(std::make_pair(length/(1+LAMBDA_SMOOTH*cost), particle_num));
      }
    }

    for (unsigned int i = 0; i < list.size(); ++i) {
      const ParticleIdx particle_num = list[i].second;
      const CvPoint2D32f p1 = pms.getPosition(frame_num, particle_num);
      double sum = 0;
      for (unsigned int j = 0; j < list.size(); ++j) {
        const CvPoint2D32f p2 = pms.getPosition(frame_num, list[j].second);
        const double dx = p1.x-p2.x;
        const double dy = p1.y-p2.y;
        const double squared_dist = dx*dx+dy*dy;
        sum += exp(-squared_dist/900);
      }
      list[i].first /= sum;
    }

    {
      sort(list.rbegin(), list.rend());
      for (unsigned int i = 0; i < FILTERED && i < list.size(); ++i) {
        const ParticleIdx particle_num = list[i].second;
        particles.push_back(particle_num);
        WeightMap[std::make_pair(frame_num, particle_num)] = 
          list[i].first;
      }
    }

    {
      double sum = 0;
      for (unsigned int i = 0; i < particles.size(); ++i) {
        sum += WeightMap[std::make_pair(frame_num, particles[i])];
      }

      for (unsigned int i = 0; i < particles.size(); ++i) {
        WeightMap[std::make_pair(frame_num, particles[i])] /= sum;
      }
    }
    filtered_particles[frame_num] = particles;

    if (filtered_particles[frame_num].size() < MIN_FILTERED) {
      std::cout << frame_num  << " has only " << filtered_particles[frame_num].size() << " tracks." <<
        std::endl;
    } 
  }
  // }}}
}

int main( int argc, char *argv[]) {
  if (argc != 5) {
    std::cout << "Usage: " << argv[0];
    std::cout << " <tracks_filename> <output_file> <lambda_scale_regur> <lambda_uncovered>";
    std::cout << std::endl;
    return -1;
  }
/*
  const char *video_filename = argv[1];
  const char *tracks_filename = argv[2];
  const char *output_filename = argv[3];
  */
  /*
  CvCapture *video = cvCreateFileCapture( video_filename );
  FRAME_WIDTH =
    cvGetCaptureProperty(video, CV_CAP_PROP_FRAME_WIDTH );

  FRAME_HEIGHT = 
    cvGetCaptureProperty(video, CV_CAP_PROP_FRAME_HEIGHT );
*/

  const char *tracks_filename = argv[1];
  const char *output_filename = argv[2];
  const char *lambda_scale_regur = argv[3];
  const char *lambda_uncovered = argv[4];

  LAMBDA_SCALE_REGUR = atof(lambda_scale_regur);
  LAMBDA_UNCOVERED = atof(lambda_uncovered);

	// XXX: 
	{
		FRAME_WIDTH = 640;
		FRAME_HEIGHT= 480;
	}
	// 

#ifndef NDEBUG
	cout << "Frame width: " << FRAME_WIDTH << endl;
	cout << "Frame heght: " << FRAME_HEIGHT<< endl;
#endif

	SIFTParticleMaps pms;
	load_tracks(pms, tracks_filename);

	vector< vector<ParticleIdx> > filtered_particles(pms.size());
	filtered_tracks(filtered_particles, pms);
	std::cout << filtered_particles[1].size() << std::endl;

	vector<SimilarityTransformMatrix> tList(pms.size());
	for (int iw = 0; iw < ( (signed int)pms.size()-WINDOW_SIZE+WINDOW_STEP-1)/WINDOW_STEP+1; ++iw) {
		const unsigned int begin_frame = iw*WINDOW_STEP+1;
		unsigned int end_frame = begin_frame+WINDOW_SIZE-1;
		if (end_frame > pms.size()-2) {
			end_frame = pms.size()-2;
		}

		//Dout( dc::notice, "From " << begin_frame << " to " << end_frame);
		std::cout << "From " << begin_frame << " to " << end_frame << std::endl;

		double last = std::numeric_limits<double>::max();
		vector<SimilarityTransformMatrix> lastList;
		for (unsigned int i = begin_frame; i <= end_frame; ++i) {
			const double sv    = 1 + SCALE_TOR;
			tList[i].sv = sv;

			const double scale =
				sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
			tList[i].a = scale;
			tList[i].b = 0;
			tList[i].dx = -(scale-1)/2*FRAME_WIDTH;
			tList[i].dy = -(scale-1)/2*FRAME_HEIGHT;
		}

		if (SCALE_TOR > 0.5) {
			while (1) {
				double result = optimizeSimilarityTransfomation(
						tList, filtered_particles, pms, begin_frame, end_frame);
				if (result >= last)
					break;

				last = result;
				lastList = tList;

				for (unsigned int i = begin_frame; i <= end_frame; ++i) {
					const double oscale = sqrt(tList[i].a*tList[i].a+tList[i].b*tList[i].b);
					const double sv    = tList[i].sv * 0.99;
					tList[i].sv = sv;

					const double scale =
						sqrt(sv*sv+SimilarityTransformMatrix::DELTA_SCALE);
					tList[i].a = tList[i].a / oscale * scale;
					tList[i].b = tList[i].b / oscale * scale;
					tList[i].dx = tList[i].dx / oscale * scale;
					tList[i].dy = tList[i].dy / oscale * scale;
				}
			}
			tList = lastList;
		} else {
			optimizeSimilarityTransfomation(
					tList, filtered_particles, pms, begin_frame, end_frame);
		}
	}

	std::ofstream fp(output_filename);
	for (unsigned int i = 0; i < pms.size(); ++i) {
		fp << tList[i].a << " " << tList[i].b << " " << tList[i].dx << " " <<
			tList[i].dy << std::endl;
	}
	fp.close();
}

