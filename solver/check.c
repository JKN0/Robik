/******************************************************************************
 * Robik - Rubik's cube solver from junk
 *
 * Color detect checker and converter.
 * Check color scanning result and try to correct minor detection errors.
 * Convert color scanning result to format suitable for solver algorithm.
 *
 * check.c
 *
 *  6.1.2018
 *
 *  12.10.2018 - Added CheckCandidate() for better color error correction
 *
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include "tinyprintf.h"

/* =====================================================================
------------------------ Constants ---------------------------------- */

#define FACES           6
#define EDGES          12
#define CORNERS         8
#define ORIENTATIONS   24

// color codes
#define RED    0
#define ORA    1
#define YEL    2
#define GRN    3
#define BLU    4
#define WHT    5

// face codes
#define FRONT  0
#define LEFT   1
#define BACK   2
#define RIGHT  3
#define UP     4
#define DOWN   5

#define NONE   6

// face bit masks
#define M_FRONT  0x01
#define M_LEFT   0x02
#define M_BACK   0x04
#define M_RIGHT  0x08
#define M_UP     0x10
#define M_DOWN   0x20
#define M_NONE   0x40

#define LEFTCOL     0
#define MIDCOL      1
#define RIGHTCOL    2

#define UPROW       0
#define MIDROW      1
#define DOWNROW     2
 
#define OK     0
#define FAIL  -1

#define ERR   99

/* =====================================================================
------------------------ Type definitions --------------------------- */

typedef struct tile_t {
    uint8_t f;          // face
    uint8_t x;          // column
    uint8_t y;          // row
} TILE_T;

/* =====================================================================
------------------------ Global variables --------------------------- */

// All possible center piece orientations
const uint8_t center_orientations[ORIENTATIONS][FACES] = {
    // F    L    B    R    U    D
    { WHT, RED, YEL, ORA, GRN, BLU },
    { RED, YEL, ORA, WHT, GRN, BLU },
    { YEL, ORA, WHT, RED, GRN, BLU },
    { ORA, WHT, RED, YEL, GRN, BLU },
    
    { WHT, ORA, YEL, RED, BLU, GRN },
    { ORA, YEL, RED, WHT, BLU, GRN },
    { YEL, RED, WHT, ORA, BLU, GRN },
    { RED, WHT, ORA, YEL, BLU, GRN },
    
    { WHT, GRN, YEL, BLU, ORA, RED },
    { GRN, YEL, BLU, WHT, ORA, RED },
    { YEL, BLU, WHT, GRN, ORA, RED },
    { BLU, WHT, GRN, YEL, ORA, RED },
    
    { WHT, BLU, YEL, GRN, RED, ORA },
    { BLU, YEL, GRN, WHT, RED, ORA },
    { YEL, GRN, WHT, BLU, RED, ORA },
    { GRN, WHT, BLU, YEL, RED, ORA },
    
    { BLU, RED, GRN, ORA, WHT, YEL },
    { RED, GRN, ORA, BLU, WHT, YEL },
    { GRN, ORA, BLU, RED, WHT, YEL },
    { ORA, BLU, RED, GRN, WHT, YEL },
    
    { BLU, ORA, GRN, RED, YEL, WHT },
    { ORA, GRN, RED, BLU, YEL, WHT },
    { GRN, RED, BLU, ORA, YEL, WHT },
    { RED, BLU, ORA, GRN, YEL, WHT },
};

// All edge cubies in face/tile coordinates
const TILE_T edge_cubies[EDGES][2] = {
    // ------- tile[0] --------    -------- tile[1] --------
    //   f       x       y           f      x       y    
    { { UP,   MIDCOL,  DOWNROW }, { FRONT,MIDCOL,  UPROW   } },    // UF
    { { UP,   RIGHTCOL,MIDROW  }, { RIGHT,MIDCOL,  UPROW   } },    // UR
    { { UP,   MIDCOL,  UPROW   }, { BACK, MIDCOL,  UPROW   } },    // UB
    { { UP,   LEFTCOL, MIDROW  }, { LEFT, MIDCOL,  UPROW   } },    // UL
    
    { { DOWN, MIDCOL,  UPROW   }, { FRONT,MIDCOL,  DOWNROW } },    // DF
    { { DOWN, RIGHTCOL,MIDROW  }, { RIGHT,MIDCOL,  DOWNROW } },    // DR
    { { DOWN, MIDCOL,  DOWNROW }, { BACK, MIDCOL,  DOWNROW } },    // DB
    { { DOWN, LEFTCOL, MIDROW  }, { LEFT, MIDCOL,  DOWNROW } },    // DL
    
    { { FRONT,RIGHTCOL,MIDROW  }, { RIGHT,LEFTCOL, MIDROW  } },    // FR
    { { FRONT,LEFTCOL, MIDROW  }, { LEFT, RIGHTCOL,MIDROW  } },    // FL

    { { BACK, LEFTCOL, MIDROW  }, { RIGHT,RIGHTCOL,MIDROW  } },    // BR
    { { BACK, RIGHTCOL,MIDROW  }, { LEFT, LEFTCOL, MIDROW  } },    // BL
};

// All corner cubies in face/tile coordinates
const TILE_T corner_cubies[CORNERS][3] = {
    // ------- tile[0] --------    -------- tile[1] -------   -------- tile[2] --------- 
    //   f       x       y           f      x       y            f      x       y    
    { { UP,   RIGHTCOL,DOWNROW }, { FRONT,RIGHTCOL,UPROW   }, { RIGHT,LEFTCOL, UPROW   } },    // UFR
    { { UP,   RIGHTCOL,UPROW   }, { RIGHT,RIGHTCOL,UPROW   }, { BACK, LEFTCOL, UPROW   } },    // URB
    { { UP,   LEFTCOL, UPROW   }, { BACK, RIGHTCOL,UPROW   }, { LEFT, LEFTCOL, UPROW   } },    // UBL
    { { UP,   LEFTCOL, DOWNROW }, { LEFT, RIGHTCOL,UPROW   }, { FRONT,LEFTCOL, UPROW   } },    // ULF
    
    { { DOWN, RIGHTCOL,UPROW   }, { RIGHT,LEFTCOL, DOWNROW }, { FRONT,RIGHTCOL,DOWNROW } },    // DRF
    { { DOWN, LEFTCOL, UPROW   }, { FRONT,LEFTCOL, DOWNROW }, { LEFT, RIGHTCOL,DOWNROW } },    // DFL
    { { DOWN, LEFTCOL, DOWNROW }, { LEFT, LEFTCOL, DOWNROW }, { BACK, RIGHTCOL,DOWNROW } },    // DLB
    { { DOWN, RIGHTCOL,DOWNROW }, { BACK, LEFTCOL, DOWNROW }, { RIGHT,RIGHTCOL,DOWNROW } },    // DBR
};

const uint8_t face_bitmaps[] = { M_FRONT, M_LEFT, M_BACK, M_RIGHT, M_UP, M_DOWN, M_NONE };

// Valid edge cubies as bitmaps
const uint8_t valid_edges[EDGES] = {
    M_UP    | M_FRONT,     // UF
    M_UP    | M_RIGHT,     // UR
    M_UP    | M_BACK,      // UB
    M_UP    | M_LEFT,      // UL
    
    M_DOWN  | M_FRONT,     // DF
    M_DOWN  | M_RIGHT,     // DR
    M_DOWN  | M_BACK,      // DB
    M_DOWN  | M_LEFT,      // DL
    
    M_FRONT | M_RIGHT,     // FR
    M_FRONT | M_LEFT,      // FL

    M_BACK  | M_RIGHT,     // BR
    M_BACK  | M_LEFT,      // BL
};

// Valid corner cubies as bitmaps
const uint8_t valid_corners[CORNERS] = {
    M_UP   | M_FRONT | M_RIGHT,     // UFR
    M_UP   | M_RIGHT | M_BACK,      // URB
    M_UP   | M_BACK  | M_LEFT,      // UBL
    M_UP   | M_LEFT  | M_FRONT,     // ULF
    
    M_DOWN | M_RIGHT | M_FRONT,     // DRF
    M_DOWN | M_FRONT | M_LEFT,      // DFL
    M_DOWN | M_LEFT  | M_BACK,      // DLB
    M_DOWN | M_BACK  | M_RIGHT,     // DBR
};

const char *colch = "ROYGBWN";
const char *facech = "FLBRUDN";

char *scanned_colors[FACES];
uint8_t color_cube[FACES][3][3];
uint8_t cube[FACES][3][3];
uint8_t candidate_cube[FACES][3][3];

extern char solver_intbl[20][4];
extern char inbuf[];

/* =====================================================================
------------------------ Function prototypes ------------------------ */

int TokenizeInbuf(void);
int ConvertScannedColors2ColorCube(void);
uint8_t GetColorCode(char c);
int FindCenterOrientation(void);
int Convert2FLBRUD( int onr );
int ConvertColor( int onr, int color );
int CheckEdges1( void );
int CheckEdges2( void );
int CheckCorners( void );
void CompareBitmaps( uint8_t *a, uint8_t *b, int *r, int len );
void GetFacesFromBitmap( uint8_t bm, uint8_t *faces, int fmax );
int CountBits( uint8_t bm );
int Convert2SolverInput(void);
void CopyToCandidate( void );
int CheckCandidate( void );

/* =====================================================================
Check color scanning result and try to correct minor detection errors.
Convert color scanning result to format suitable for solver algorithm.

Before call, inbuf[] contains all faces' scanned colors as a string like
"GOB-BGG-BOO GYY-WRY-GBW OGW-RBG-GBW ROO-BGN-BYR BWY-GOW-RYY RRW-RYO-OWY".

After the call, the solver_intbl[] contains corresponding data as string
table, like

solver_intbl[0]:"DR"
solver_intbl[1]:"RF"
solver_intbl[2]:"LF"
solver_intbl[3]:"LU"
solver_intbl[4]:"UR"
solver_intbl[5]:"BD"
solver_intbl[6]:"RB"
solver_intbl[7]:"LB"
solver_intbl[8]:"FU"
solver_intbl[9]:"BU"
solver_intbl[10]:"LD"
solver_intbl[11]:"FD"
solver_intbl[12]:"UBL"
solver_intbl[13]:"DBR"
solver_intbl[14]:"LDF"
solver_intbl[15]:"RUF"
solver_intbl[16]:"BUR"
solver_intbl[17]:"LBD"
solver_intbl[18]:"RFD"
solver_intbl[19]:"LFU"
--------------------------------------------------------------------- */

int CheckColors( void )
{
    int co_nr;
    int rc;

    rc = TokenizeInbuf();
    if (rc == FAIL)
        return 1;

    rc = ConvertScannedColors2ColorCube();
    if (rc == FAIL)
        return 2;

    co_nr = FindCenterOrientation();
    if (co_nr == FAIL)
        return 3;
        
    rc = Convert2FLBRUD(co_nr);
    if (rc == FAIL)
        return 4;

    rc = CheckEdges1();
    if (rc == FAIL)
    {
        // try another way
    	rc = CheckEdges2();
        if (rc == FAIL)
            return 5;
    }

    rc = CheckCorners();
    if (rc == FAIL)
        return 6;
    
    CopyToCandidate();
    rc = CheckCandidate();
    if (rc == FAIL)
        return 7;

    rc = Convert2SolverInput();
    if (rc == FAIL)
        return 8;
    
    return 0;
}

/* =====================================================================
Tokenize inbuf[] to scanned_colors[].
inbuf contains all faces' scanned colors. The faces are in order FLBDRU 
instead of normal FLBRUD. This order is converted here.
Each face in inbuf is a string in format like "YYW-RGW-YBG", faces are 
separated by a space.
Each face in scanned_colors[] is a pointer to one face string.
------------------------------------------------------------------------ */

int TokenizeInbuf(void)
{
    const int face_order[] = { 0,1,2,5,3,4 };   // FLBDRU

    char *p,*q;
    int i;

    p = inbuf;
    for (i = 0; i < FACES; i++)
    {
        q = strchr(p,' ');
        if (q == NULL)
        	break;
        else
            *q = '\0';

        scanned_colors[face_order[i]] = p;
        p = q + 1;
    }

    if (i != FACES)
    	return FAIL;

    return OK;
}

/* =====================================================================
Convert scanned_colors[] to color_cube[].
Each face in scanned_colors[] is a string in format like "YYW-RGW-YBG".
Each face in color_cube[] is a 3*3 matrix of color codes (RED,BLU,...)
Color scanner scans DOWN and RIGHT faces in different order than others, 
this is taken into account here. Orders in strings are:
   0  1  2 3  4 5  6 7  8  9 10
  LU MU RU - LM C RM - LD MD RD  normal
  RD MD LD - RM C LM - RU MU LU  down
  RU RM RD - MU C MD - LU LM LD  right
------------------------------------------------------------------------ */

int ConvertScannedColors2ColorCube(void)
{
    const int scan_order_normal[] = { 0,1,2,  4,5,6, 8,9,10 }; 
    const int scan_order_down[]   = { 10,9,8, 6,5,4, 2,1,0  }; 
    const int scan_order_right[]  = { 8,4,0,  9,5,1, 10,6,2 }; 
    
    int f,x,y;
    int sc_idx;
    char col_ch;
    const int *scan_order;
    
    // for all faces
    for (f = 0; f < FACES; f++)
    {
        // different scan order for D and R faces
        switch (f)
        {
        case DOWN:  scan_order = scan_order_down;   break;
        case RIGHT: scan_order = scan_order_right;  break;
        default:    scan_order = scan_order_normal; break;
        }
        
        // convert one face scan result to one face in color_cube[]
        sc_idx = 0;
        for (y = 0; y < 3; y++)
        {
            for (x = 0; x < 3; x++)
            {
                col_ch = scanned_colors[f][scan_order[sc_idx]];
                color_cube[f][x][y] = GetColorCode(col_ch);
                if (color_cube[f][x][y] == ERR)
                    return FAIL;
                sc_idx++;
            }
        }       
    }
    
    return OK;
}

/* =====================================================================
Return color code corrsponding to color letter.
------------------------------------------------------------------------ */

uint8_t GetColorCode(char c)
{
    switch (c)
    {
    case 'R': return RED;
    case 'O': return ORA;
    case 'Y': return YEL;
    case 'G': return GRN;
    case 'B': return BLU;
    case 'W': return WHT;
    case 'N': return NONE;
    default: return ERR;
    }
}
    
/* =====================================================================
Find orientation of center pieces. There are 24 possible orientations,
return orientation number (index to center_orientations[]).
Some center colors (at least white) may be detected incorrectly. 
This works despite of any one color mistake and partly with two color mistakes.
------------------------------------------------------------------------ */

int FindCenterOrientation(void)
{
    int match_ctr[ORIENTATIONS];
    int max_ctr_val = 0;
    int best_onr = -1;
    int max_cnt = 0;
    int onr,f;
    
    // count how many colors match to each orientation
    for (onr = 0; onr < ORIENTATIONS; onr++)
    {
        match_ctr[onr] = 0;
        
        for (f = 0; f < FACES; f++)
        {
            if (color_cube[f][1][1] == center_orientations[onr][f])
                match_ctr[onr]++;
        }
    }
    
    // find best match (highest match count)
    for (onr = 0; onr < ORIENTATIONS; onr++)
    {
        if (match_ctr[onr] > max_ctr_val)
        {
            max_ctr_val = match_ctr[onr];
            best_onr = onr;
        }
    }    

    // is the best match unique?
    for (onr = 0; onr < ORIENTATIONS; onr++)
    {
        if (match_ctr[onr] == max_ctr_val)
            max_cnt++;
    }    

    if (max_cnt == 1)
        return best_onr;
    else
        return FAIL;  // best match not unique, cannot resolve
}

/* =====================================================================
Convert cube from color codes to FLBRUD-reperesentation according to
current orientation.
After this, each face in cube[] contains a 3*3 matrix of face codes (FRONT,LEFT,...)
------------------------------------------------------------------------ */

int Convert2FLBRUD( int onr )
{
    int f,x,y;
    
    for (f = 0; f < FACES; f++)
    {
        for (y = 0; y < 3; y++)
        {
            for (x = 0; x < 3; x++)
            {
                if (x == 1 && y == 1)
                    cube[f][x][y] = ConvertColor(onr,center_orientations[onr][f]);   // correct center color detection mistakes
                else
                    cube[f][x][y] = ConvertColor(onr,color_cube[f][x][y]);
                
                if (cube[f][x][y] == ERR)
                    return FAIL;
            }        
        }        
    }
    
    return OK;
}

/* =====================================================================
Convert color code to FLBRUD-reperesentation according to current orientation.
------------------------------------------------------------------------ */

int ConvertColor( int onr, int color )
{
    int f;
    
    if (color == NONE)
        return NONE;
    
    for (f = 0; f < FACES; f++)
    {
        if (center_orientations[onr][f] == color)
            return f;
    }    
    
    return ERR;
}

/* =====================================================================
Check edge cubies: check that all valid edge cubies exist exactly once.
Certain types of detection errors are corrected, most of the errors cannot
be corrected.
------------------------------------------------------------------------ */

int CheckEdges1( void )
{
    int f,x,y;
    int i,e,t;
    uint8_t cube_edges[EDGES];
    uint8_t valid_faces[2];
    int c_in_v[EDGES];
    int v_in_c[EDGES];
    int ie,me,de;
    int dup[2];
    int cv0=0,cv1=0,cvm=0;
    int vc0=0,vc1=0,vc2=0,vcm=0;
    int ok_cand_ctr = 0;
    int ok_cand = 0;
    uint8_t diff;
    uint8_t wf=0,rf=0;
    int t_corr,ok_tile;

    // build edge bitmap table from cube
    for (e = 0; e < EDGES; e++)
    {
        cube_edges[e] = 0;
        for (t = 0; t < 2; t++)
        {
            f = edge_cubies[e][t].f;
            x = edge_cubies[e][t].x;
            y = edge_cubies[e][t].y;
            cube_edges[e] |= face_bitmaps[cube[f][x][y]];
        }
    }
    
    // compare cube bitmaps to valid bitmaps in both directions
    CompareBitmaps(cube_edges,(uint8_t *)valid_edges,c_in_v,EDGES);
    CompareBitmaps((uint8_t *)valid_edges,cube_edges,v_in_c,EDGES);
    
    // count 0, 1 and multiple hits in both tables
    for (e = 0; e < EDGES; e++)
    {
        switch (c_in_v[e])
        {
        case 0:  cv0++; break;
        case 1:  cv1++; break;
        default: cvm++; break;
        } 
        
        switch (v_in_c[e])
        {
        case 0:  vc0++; break;
        case 1:  vc1++; break;
        case 2:  vc2++; break;
        default: vcm++; break;
        }
    }    
    
    // if all hit counts are 1, there are no errors
    if (cv1 == EDGES && vc1 == EDGES)
        return OK;
    
    // if any hit count is > 2, error correction is not possible
    if (cvm != 0 || vcm != 0)
        return FAIL;
    
    /* There are two detection error cases that can be corrected:

        1. Cube has one invalid cubie (a cubie which cannot be found from
           valid cubies table) and one valid cubie is missing from the cube
           (one valid cubie cannot be found from the cube). Then the invalid
           cubie should be replaced with the missing valid cubie.

           The "invalidness" of the cubie may consist of 1 or 2 misdetected
           colors, because we will replace the whole cubie.

        2. Cube has two identical cubies (one valid cubie is found twice from
           the cube) and one valid cubie is missing (one valid cubie cannot
           be found from the cube). Then one of the identical cubies should
           be replaced with the missing valid cubie.

           Assumption is that only one of the identical cubies has one
           misdetected color, thus we will replace only one tile. We cannot
           replace the whole cubie, because we may get more than one solvable
           candidate while trying out various combinations, and would have
           to discard the whole correction.
    */

    // --- Case 1: one invalid and one missing
    if (cv0 == 1 && vc0 == 1)
    {
        // find the 0 in both tables
        for (e = 0; e < EDGES; e++)
        {
            if (c_in_v[e] == 0)
                ie = e;     // invalid edge

            if (v_in_c[e] == 0)
                me = e;     // missing edge
        }

        // copy the face list of missing valid cubie to valid_faces[]
        for (i = 0; i < 2; i++)
            valid_faces[i] = edge_cubies[me][i].f;

        // replace the invalid cubie with valid one
        // replacement is done two times, in both cubie orientations, and both are validated
        for (i = 0; i < 2; i++)
        {
            // reset candidate
            CopyToCandidate();

            // replace both tiles from valid faces
            f = edge_cubies[ie][0].f;
            x = edge_cubies[ie][0].x;
            y = edge_cubies[ie][0].y;
            candidate_cube[f][x][y] = valid_faces[i];
            
            f = edge_cubies[ie][1].f;
            x = edge_cubies[ie][1].x;
            y = edge_cubies[ie][1].y;
            candidate_cube[f][x][y] = valid_faces[1-i];

            if (CheckCandidate() == OK)
            {
                ok_cand_ctr++;
                ok_cand = i;
            }
        }

        // exactly one candidate should be solvable
        if (ok_cand_ctr != 1)
            return FAIL;

        // redo the correct replacement in actual cube
        f = edge_cubies[ie][0].f;
        x = edge_cubies[ie][0].x;
        y = edge_cubies[ie][0].y;
        cube[f][x][y] = valid_faces[ok_cand];

        f = edge_cubies[ie][1].f;
        x = edge_cubies[ie][1].x;
        y = edge_cubies[ie][1].y;
        cube[f][x][y] = valid_faces[1-ok_cand];

        return OK;
    }

    // --- Case 2: two identical and one missing
    if (cv1 == EDGES && vc0 == 1 && vc2 == 1)
    {
        // find the missing one and duplicate index
        for (e = 0; e < EDGES; e++)
        {
            if (v_in_c[e] == 0)
                me = e;     // missing edge

            if (v_in_c[e] == 2)
                de = e;     // duplicate edge
        }

        // find the duplicate cubies positions in cube
        i = 0;
        for (e = 0; e < EDGES; e++)
        {
            if (cube_edges[e] == valid_edges[de])
            {
                dup[i] = e;
                i++;
            }
        }
        
        // find differences in colors
        // use only dup[0] here because dup[0] and dup[1] are identical cubies
        diff = cube_edges[dup[0]] ^ valid_edges[me];

        // if only one difference or more than two => cannot correct
        if (CountBits(diff) != 2)
            return FAIL;
        
        // (diff & cube_edges[dup[0]]) is the bitmap of the wrong face code in the cube
        GetFacesFromBitmap(diff & cube_edges[dup[0]],&wf,1);
        
        // (diff & valid_edges[me]) is the bitmap of the right face code
        // which should be in place of the wrong tile
        GetFacesFromBitmap(diff & valid_edges[me],&rf,1);
        
        // try to replace the wrong tile with the right face code
        // replacement is done for one of the identical cubies at a time
        for (i = 0; i < 2; i++)
        {
            // reset candidate
        	CopyToCandidate();
            
            // search the wrong tile from the cube's cubie
            for (t = 0; t < 2; t++)
            {
                f = edge_cubies[dup[i]][t].f;
                x = edge_cubies[dup[i]][t].x;
                y = edge_cubies[dup[i]][t].y;

                if (candidate_cube[f][x][y] == wf)
                {
                    // correct the error in candidate
                    candidate_cube[f][x][y] = rf;
                    t_corr = t;
                }
            }

            // if candidate is solvable, this was the right cubie
            if (CheckCandidate() == OK)
            {
                ok_cand_ctr++;
                ok_cand = i;
                ok_tile = t_corr;
            }
        }

        // exactly one candidate should be solvable
        if (ok_cand_ctr != 1)
            return FAIL;

        // redo the correct replacement in actual cube
        f = edge_cubies[dup[ok_cand]][ok_tile].f;
        x = edge_cubies[dup[ok_cand]][ok_tile].x;
        y = edge_cubies[dup[ok_cand]][ok_tile].y;
        cube[f][x][y] = rf;

        return OK;
    }    
    
    return FAIL;    // other cases => cannot correct
}

/* =====================================================================
Another way to correct edge cubies. In certain cases this may succeed,
although CheckEdges1() has failed.
------------------------------------------------------------------------ */

int CheckEdges2( void )
{
    int f,x,y;
    int i,e;
    uint8_t cube_edges[EDGES];
    int c_in_v[EDGES];
    int v_in_c[EDGES];
    int ie,me;
    int cv0=0,cv1=0,cvm=0;
    int vc0=0,vc1=0,vcm=0;
    uint8_t diff;
    uint8_t wf=0,rf=0;

    // build edge bitmap table from cube
    for (e = 0; e < EDGES; e++)
    {
        cube_edges[e] = 0;
        for (i = 0; i < 2; i++)
        {
            f = edge_cubies[e][i].f;
            x = edge_cubies[e][i].x;
            y = edge_cubies[e][i].y;
            cube_edges[e] |= face_bitmaps[cube[f][x][y]];
        }
    }

    // compare cube bitmaps to valid bitmaps in both directions
    CompareBitmaps(cube_edges,(uint8_t *)valid_edges,c_in_v,EDGES);
    CompareBitmaps((uint8_t *)valid_edges,cube_edges,v_in_c,EDGES);

    // count 0, 1 and multiple hits in both tables
    for (e = 0; e < EDGES; e++)
    {
        switch (c_in_v[e])
        {
        case 0:  cv0++; break;
        case 1:  cv1++; break;
        default: cvm++; break;
        }

        switch (v_in_c[e])
        {
        case 0:  vc0++; break;
        case 1:  vc1++; break;
        default: vcm++; break;
        }
    }

    // if all hit counts are 1, there are no errors
    if (cv1 == EDGES && vc1 == EDGES)
        return OK;

    // if any hit count is > 1, error correction is not possible
    if (cvm != 0 || vcm != 0)
        return FAIL;

    // if both tables have one 0, error may be possible to correct
    if (cv0 == 1 && vc0 == 1)
    {
        // find the 0 in both tables
        for (e = 0; e < EDGES; e++)
        {
            if (c_in_v[e] == 0)
                ie = e;

            if (v_in_c[e] == 0)
                me = e;
        }

        // find differences in colors
        diff = cube_edges[ie] ^ valid_edges[me];

        // if only one difference or more than two => cannot correct
        if (CountBits(diff) != 2)
            return FAIL;

        // (diff & cube_edges[ie]) is the bitmap of the wrong face code in the cube
        GetFacesFromBitmap(diff & cube_edges[ie],&wf,1);

        // (diff & valid_edges[me]) is the bitmap of the right face code that
        // should be in place of the wrong tile
        GetFacesFromBitmap(diff & valid_edges[me],&rf,1);

        // search the wrong tile from the cube's cubie
        for (i = 0; i < 2; i++)
        {
            f = edge_cubies[ie][i].f;
            x = edge_cubies[ie][i].x;
            y = edge_cubies[ie][i].y;

            if (cube[f][x][y] == wf)
            {
                // correct the error
                cube[f][x][y] = rf;
                return OK;
            }
        }
    }

    return FAIL;    // other cases => cannot correct
}
    
/* =====================================================================
Check corner cubies: check that all valid corner cubies exist exactly once.
Certain types of detection errors are corrected, most of the errors cannot
be corrected.
------------------------------------------------------------------------ */

int CheckCorners( void )
{
    int f,x,y;
    int i,c,t;
    uint8_t cube_corners[CORNERS];
    uint8_t valid_faces[3];
    int c_in_v[CORNERS];
    int v_in_c[CORNERS];
    int ic,mc,dc;
    int dup[2];
    int cv0=0,cv1=0,cvm=0;
    int vc0=0,vc1=0,vc2=0,vcm=0;
    int ok_cand_ctr = 0;
    int ok_cand = 0;
    uint8_t diff;
    uint8_t wf=0,rf=0;
    int t_corr,ok_tile;

    // build corner bitmap table from cube
    for (c = 0; c < CORNERS; c++)
    {
        cube_corners[c] = 0;
        for (i = 0; i < 3; i++)
        {
            f = corner_cubies[c][i].f;
            x = corner_cubies[c][i].x;
            y = corner_cubies[c][i].y;
            cube_corners[c] |= face_bitmaps[cube[f][x][y]];
        }
    }
    
    // compare cube bitmaps to valid bitmaps in both directions
    CompareBitmaps(cube_corners,(uint8_t *)valid_corners,c_in_v,CORNERS);
    CompareBitmaps((uint8_t *)valid_corners,cube_corners,v_in_c,CORNERS);
    
    // count 0, 1 and multiple hits in both tables
    for (c = 0; c < CORNERS; c++)
    {
        switch (c_in_v[c])
        {
        case 0:  cv0++; break;
        case 1:  cv1++; break;
        default: cvm++; break;
        } 
        
        switch (v_in_c[c])
        {
        case 0:  vc0++; break;
        case 1:  vc1++; break;
        case 2:  vc2++; break;
        default: vcm++; break;
        }
    }    
    
    // if all hit counts are 1, there are no errors
    if (cv1 == CORNERS && vc1 == CORNERS)
        return OK;

    // if any hit count is > 2, error correction is not possible
    if (cvm != 0 || vcm != 0)
        return FAIL;
    
    /* There are two detection error cases that can be corrected:

        1. Cube has one invalid cubie (a cubie which cannot be found from
           valid cubies table) and one valid cubie is missing from the cube
           (one valid cubie cannot be found from the cube). Then the invalid
           cubie should be replaced with the missing valid cubie.

           The "invalidness" of the cubie may consist of 1...3 misdetected
           colors, because we will replace the whole cubie.

        2. Cube has two identical cubies (one valid cubie is found twice from
           the cube) and one valid cubie is missing (one valid cubie cannot
           be found from the cube). Then one of the identical cubies should
           be replaced with the missing valid cubie.

           Assumption is that only one of the identical cubies has one
           misdetected color, thus we will replace only one tile. We cannot
           replace the whole cubie, because we may get more than one solvable
           candidate while trying out various combinations, and would have
           to discard the whole correction.
    */

    // --- Case 1: one invalid and one missing
    if (cv0 == 1 && vc0 == 1)
    {
        // find the 0 in both tables
        for (c = 0; c < CORNERS; c++)
        {
            if (c_in_v[c] == 0)
               ic = c;      // invalid corner

            if (v_in_c[c] == 0)
                mc = c;     // missing corner
        }

        // copy the face list of missing valid cubie to valid_faces[]
        for (i = 0; i < 3; i++)
            valid_faces[i] = corner_cubies[mc][i].f;

        // replace the invalid cubie with valid one
        // replacement is done 3 times, in all cubie orientations, and all are validated
        for (i = 0; i < 3; i++)
        {
            // reset candidate
            CopyToCandidate();

            // replace all tiles from valid faces
            f = corner_cubies[ic][0].f;
            x = corner_cubies[ic][0].x;
            y = corner_cubies[ic][0].y;
            candidate_cube[f][x][y] = valid_faces[i];

            f = corner_cubies[ic][1].f;
            x = corner_cubies[ic][1].x;
            y = corner_cubies[ic][1].y;
            candidate_cube[f][x][y] = valid_faces[(i+1)%3];

            f = corner_cubies[ic][2].f;
            x = corner_cubies[ic][2].x;
            y = corner_cubies[ic][2].y;
            candidate_cube[f][x][y] = valid_faces[(i+2)%3];

           if (CheckCandidate() == OK)
            {
                ok_cand_ctr++;
                ok_cand = i;
            }
        }

        // exactly one candidate should be solvable
        if (ok_cand_ctr != 1)
            return FAIL;

        // redo the correct replacement in actual cube
        f = corner_cubies[ic][0].f;
        x = corner_cubies[ic][0].x;
        y = corner_cubies[ic][0].y;
        cube[f][x][y] = valid_faces[ok_cand];

        f = corner_cubies[ic][1].f;
        x = corner_cubies[ic][1].x;
        y = corner_cubies[ic][1].y;
        cube[f][x][y] = valid_faces[(ok_cand+1)%3];

        f = corner_cubies[ic][2].f;
        x = corner_cubies[ic][2].x;
        y = corner_cubies[ic][2].y;
        cube[f][x][y] = valid_faces[(ok_cand+2)%3];

        return OK;
    }

    // --- Case 2: two identical and one missing
    if (cv1 == CORNERS && vc0 == 1 && vc2 == 1)
    {
        // find the missing one and duplicate index
        for (c = 0; c < CORNERS; c++)
        {
            if (v_in_c[c] == 0)
                mc = c;     // missing corner

            if (v_in_c[c] == 2)
                dc = c;     // duplicate corner
        }

        // find the duplicate cubies positions in cube
        i = 0;
        for (c = 0; c < CORNERS; c++)
        {
            if (cube_corners[c] == valid_corners[dc])
            {
                dup[i] = c;
                i++;
            }
        }
        
        // find differences in colors
        // use only dup[0] here because dup[0] and dup[1] are identical cubies
        diff = cube_corners[dup[0]] ^ valid_corners[mc];

        // if only one difference or more than two => cannot correct
        if (CountBits(diff) != 2)
            return FAIL;
        
        // (diff & cube_corners[dup[0]]) is the bitmap of the wrong face code in the cube
        GetFacesFromBitmap(diff & cube_corners[dup[0]],&wf,1);
        
        // (diff & valid_corners[me]) is the bitmap of the right face code
        // which should be in place of the wrong tile
        GetFacesFromBitmap(diff & valid_corners[mc],&rf,1);

        // try to replace the wrong tile with the right face code
        // replacement is done for one of the identical cubies at a time
        for (i = 0; i < 2; i++)
        {
            // reset candidate
            CopyToCandidate();
            
            // search the wrong tile from the cube's cubie
            for (t = 0; t < 3; t++)
            {
                f = corner_cubies[dup[i]][t].f;
                x = corner_cubies[dup[i]][t].x;
                y = corner_cubies[dup[i]][t].y;

                if (candidate_cube[f][x][y] == wf)
                {
                    // correct the error in candidate
                    candidate_cube[f][x][y] = rf;
                    t_corr = t;
                }
            }

            // if candidate is solvable, this was the right cubie
            if (CheckCandidate() == OK)
            {
                ok_cand_ctr++;
                ok_cand = i;
                ok_tile = t_corr;
            }
        }

        // exactly one candidate should be solvable
        if (ok_cand_ctr != 1)
            return FAIL;

        // redo the correct replacement in actual cube
        f = corner_cubies[dup[ok_cand]][ok_tile].f;
        x = corner_cubies[dup[ok_cand]][ok_tile].x;
        y = corner_cubies[dup[ok_cand]][ok_tile].y;
        cube[f][x][y] = rf;

        return OK;
    }
    
    return FAIL;    // other cases => cannot correct
}
    
/* =====================================================================
Compare two bitmap tables a and b, and return hit count in r for 
each item in a. Parameter len is the length for all tables.
------------------------------------------------------------------------ */

void CompareBitmaps( uint8_t *a, uint8_t *b, int *r, int len )
{
    int i,j;
    
    for (i = 0; i < len; i++)
    {
        r[i] = 0;
        for (j = 0; j < len; j++)
        {
            if (a[i] == b[j])
                r[i]++;
        }
    }
}

/* =====================================================================
Return face list corresponding the given bitmap. List is fmax faces long.
------------------------------------------------------------------------ */

void GetFacesFromBitmap( uint8_t bm, uint8_t *faces, int fmax )
{
    uint8_t f;
    int i = 0;
    
    // FACES+1 == include NONE also
    for (f = 0; f < FACES+1; f++)
    {
        if ((bm & face_bitmaps[f]) != 0)
        {
            if (i < fmax)
                faces[i] = f;
            i++;
        }
    }
}

/* =====================================================================
Count number of 1-bits in byte.
------------------------------------------------------------------------ */

int CountBits( uint8_t b )
{
    int i,cnt = 0;
    
    for (i = 0; i < 7; i++)
    {
        if (b & (1<<i))
            cnt++;
    }
    
    return cnt;
}

/* =====================================================================
Convert cube[] to solver_intbl[].
After this, solver_intbl[] contains strings like
"DR"
"RF"
"LF"
"LU"
...
------------------------------------------------------------------------ */

int Convert2SolverInput(void)
{
    int i,s,e,c;
    int f,x,y;
    
    s = 0;
    
    for (e = 0; e < EDGES; e++)
    {
        for (i = 0; i < 2; i++)
        {
            f = edge_cubies[e][i].f;
            x = edge_cubies[e][i].x;
            y = edge_cubies[e][i].y;
            solver_intbl[s][i] = facech[cube[f][x][y]];
        }
        solver_intbl[s][i] = '\0';

        s++;
    }

    for (c = 0; c < CORNERS; c++)
    {
        for (i = 0; i < 3; i++)
        {
            f = corner_cubies[c][i].f;
            x = corner_cubies[c][i].x;
            y = corner_cubies[c][i].y;
            solver_intbl[s][i] = facech[cube[f][x][y]];
        }
        solver_intbl[s][i] = '\0';

        s++;
    }
    
    return OK;
}

/* =====================================================================
Copy cube[] -> candidate_cube[].
------------------------------------------------------------------------ */

void CopyToCandidate( void )
{
    int f,x,y;

    for (f = 0; f < FACES; f++)
    {
        for (y = 0; y < 3; y++)
        {
            for (x = 0; x < 3; x++)
            {
                candidate_cube[f][x][y] = cube[f][x][y];
            }
        }
    }
}


/************************************************************************
  Cube checking algorithm. Check if candidate_cube is valid, not only
  that all cubies are found somewhere in the cube, but they are also
  oriented so that the cube is solvable, i.e. the orientation is result
  of a valid move sequence.

  This is used when trying to fix color recognition errors for evaluating
  various correction candidates.

  Algorithm taken from Christoph Bandelow: Inside Rubik's cube and beyond
  pages 99...100.
  https://maths-people.anu.edu.au/~burkej/cube/bandelow.pdf

 */

// Strange variable and function names come from the book's algorithm, which
// is not in any programming language but resembles very basic BASIC.

uint8_t cube2[FACES][3][3];
uint8_t C[CORNERS+1][4];	// 1-based indexing used
uint8_t E[EDGES+1][3];
int PC[CORNERS+1];
int PE[EDGES+1];
int Z,ZC,ZE,IC,IE;

int Build_C_E( void );
void NICS( void );
void CountZC( void );
void CountZE( void );
int Build_PC_PE( void );
int SSC( uint8_t a, uint8_t b, uint8_t d );
int SSE( uint8_t a, uint8_t b );
void Count_Z( void );

/* =====================================================================
Candidate check main function. Check whether the candicate_cube is solvable.
Return OK or FAIL.
------------------------------------------------------------------------ */

int CheckCandidate( void )
{
    int rc;

    Z = 0;
    ZC = 0;
    ZE = 0;
    IC = 0;
    IE = 0;

    // Build corner and edge tables to be used in evaluation
    Build_C_E();

    // Count variables ZC and ZE
    CountZC();
    CountZE();

    // Build tables PC and PE
    // If this fails, cube is not valid at all (missing or duplicate cubies)
    rc = Build_PC_PE();
    if (rc == FAIL)
        return FAIL;

    // Count variable Z
    Count_Z();

    // For valid cube, Z and ZE must be even and ZC multiple of 3
    if ((Z % 2) == 0 && (ZE % 2) == 0 && (ZC % 3) == 0)
        return OK;

    return FAIL;
}

/* =====================================================================
Copy candidate_cube[] -> cube2[]. Table building rotates cube2[] so we don't
want to destroy the candidate_cube.
------------------------------------------------------------------------ */

void CopyCube2( void )
{
    int f,x,y;

    for (f = 0; f < FACES; f++)
    {
        for (y = 0; y < 3; y++)
        {
            for (x = 0; x < 3; x++)
            {
                cube2[f][x][y] = candidate_cube[f][x][y];
            }
        }
    }
}

/* =====================================================================
Build C[] and E[] tables
------------------------------------------------------------------------ */

int Build_C_E( void )
{
    int i,i1,i2,i3,i4,i5;

    CopyCube2();

    for (i = 1; i <= 4; i++)
    {
        i1 = 2*i - 1;
        i2 = 2*i;
        i3 = 3*i - 2;
        i4 = 3*i - 1;
        i5 = 3*i;

        // corner UFL
        C[i1][1] = cube2[UP   ][LEFTCOL ][DOWNROW];
        C[i1][2] = cube2[FRONT][LEFTCOL ][UPROW  ];
        C[i1][3] = cube2[LEFT ][RIGHTCOL][UPROW  ];

        // corner DLF
        C[i2][1] = cube2[DOWN ][LEFTCOL ][UPROW  ];
        C[i2][2] = cube2[LEFT ][RIGHTCOL][DOWNROW];
        C[i2][3] = cube2[FRONT][LEFTCOL] [DOWNROW];

        // edge UF
        E[i3][1] = cube2[UP   ][MIDCOL  ][DOWNROW];
        E[i3][2] = cube2[FRONT][MIDCOL  ][UPROW  ];

        // edge LF
        E[i4][1] = cube2[LEFT ][RIGHTCOL][MIDROW ];
        E[i4][2] = cube2[FRONT][LEFTCOL ][MIDROW ];

        // edge DF
        E[i5][1] = cube2[DOWN  ][MIDCOL][UPROW  ];
        E[i5][2] = cube2[FRONT ][MIDCOL][DOWNROW];

        // rotate cube
        NICS();
    }

    return OK;
}

/* =====================================================================
NICS: rotate cube2[] around UP-DOWN axis one quarter turn clockwise
------------------------------------------------------------------------ */

#define COPY_FACE(sf,df) {\
    int x,y;\
    for (y = 0; y < 3; y++)\
    {\
        for (x = 0; x < 3; x++)\
        {\
            df[x][y] = sf[x][y];\
        }\
    }\
}

void NICS( void )
{
    uint8_t tf[3][3];
    uint8_t tt;

    // swap faces F->L->B->R->F
    COPY_FACE(cube2[FRONT],tf);
    COPY_FACE(cube2[RIGHT],cube2[FRONT]);
    COPY_FACE(cube2[BACK],cube2[RIGHT]);
    COPY_FACE(cube2[LEFT],cube2[BACK]);
    COPY_FACE(tf,cube2[LEFT]);

    // -- rotate face U clockwise:
    // swap corners
    tt = cube2[UP][LEFTCOL][UPROW];
    cube2[UP][LEFTCOL][UPROW] = cube2[UP][LEFTCOL][DOWNROW];
    cube2[UP][LEFTCOL][DOWNROW] = cube2[UP][RIGHTCOL][DOWNROW];
    cube2[UP][RIGHTCOL][DOWNROW] = cube2[UP][RIGHTCOL][UPROW];
    cube2[UP][RIGHTCOL][UPROW] = tt;

    // swap edges
    tt = cube2[UP][LEFTCOL][MIDROW];
    cube2[UP][LEFTCOL][MIDROW] = cube2[UP][MIDCOL][DOWNROW];
    cube2[UP][MIDCOL][DOWNROW] = cube2[UP][RIGHTCOL][MIDROW];
    cube2[UP][RIGHTCOL][MIDROW] = cube2[UP][MIDCOL][UPROW];
    cube2[UP][MIDCOL][UPROW] = tt;

    // rotate face D counter-clockwise:
    // swap corners
    tt = cube2[DOWN][LEFTCOL][UPROW];
    cube2[DOWN][LEFTCOL][UPROW] = cube2[DOWN][RIGHTCOL][UPROW];
    cube2[DOWN][RIGHTCOL][UPROW] = cube2[DOWN][RIGHTCOL][DOWNROW];
    cube2[DOWN][RIGHTCOL][DOWNROW] = cube2[DOWN][LEFTCOL][DOWNROW];
    cube2[DOWN][LEFTCOL][DOWNROW] = tt;

    // swap edges
    tt = cube2[DOWN][LEFTCOL][MIDROW];
    cube2[DOWN][LEFTCOL][MIDROW] = cube2[DOWN][MIDCOL][UPROW];
    cube2[DOWN][MIDCOL][UPROW] = cube2[DOWN][RIGHTCOL][MIDROW];
    cube2[DOWN][RIGHTCOL][MIDROW] = cube2[DOWN][MIDCOL][DOWNROW];
    cube2[DOWN][MIDCOL][DOWNROW] = tt;
}

/* =====================================================================
Count ZC
------------------------------------------------------------------------ */

void CountZC( void )
{
    int i;
    uint8_t a;

    for (i = 1; i <= CORNERS; i++)
    {
        if (C[i][2] == candidate_cube[UP  ][MIDCOL][MIDROW] ||
            C[i][2] == candidate_cube[DOWN][MIDCOL][MIDROW])
        {
            a = C[i][1];
            C[i][1] = C[i][2];
            C[i][2] = C[i][3];
            C[i][3] = a;
            ZC++;
        }
        else if (C[i][3] == candidate_cube[UP  ][MIDCOL][MIDROW] ||
                 C[i][3] == candidate_cube[DOWN][MIDCOL][MIDROW])
        {
            a = C[i][1];
            C[i][1] = C[i][3];
            C[i][3] = C[i][2];
            C[i][2] = a;
            ZC += 2;
        }
    }
}

/* =====================================================================
Count ZE
------------------------------------------------------------------------ */

void CountZE( void )
{
    int i;
    uint8_t a;

    for (i = 1; i <= EDGES; i++)
    {
        if ( E[i][2] == candidate_cube[UP   ][MIDCOL][MIDROW] ||
             E[i][2] == candidate_cube[DOWN ][MIDCOL][MIDROW] ||
            (E[i][1] == candidate_cube[FRONT][MIDCOL][MIDROW] && E[i][2] == candidate_cube[LEFT ][MIDCOL][MIDROW]) ||
            (E[i][1] == candidate_cube[RIGHT][MIDCOL][MIDROW] && E[i][2] == candidate_cube[FRONT][MIDCOL][MIDROW]) ||
            (E[i][1] == candidate_cube[BACK ][MIDCOL][MIDROW] && E[i][2] == candidate_cube[RIGHT][MIDCOL][MIDROW]) ||
            (E[i][1] == candidate_cube[LEFT ][MIDCOL][MIDROW] && E[i][2] == candidate_cube[BACK ][MIDCOL][MIDROW]))
        {
            a = E[i][1];
            E[i][1] = E[i][2];
            E[i][2] = a;
            ZE++;
        }
    }
}

/* =====================================================================
Build PC[] and PE[] tables
------------------------------------------------------------------------ */

int Build_PC_PE( void )
{
    int i,rc;
    uint8_t a,b,d;

    CopyCube2();

    for (i = 1; i <= 4; i++)
    {
        a = cube2[UP   ][MIDCOL][MIDROW];
        b = cube2[FRONT][MIDCOL][MIDROW];
        d = cube2[LEFT ][MIDCOL][MIDROW];
        rc = SSC(a,b,d);
        if (rc == FAIL)
            return FAIL;

        rc = SSE(a,b);
        if (rc == FAIL)
            return FAIL;

        a = cube2[LEFT ][MIDCOL][MIDROW];
        rc = SSE(a,b);
        if (rc == FAIL)
            return FAIL;

        a = cube2[DOWN ][MIDCOL][MIDROW];
        rc = SSE(a,b);
        if (rc == FAIL)
            return FAIL;

        b = cube2[LEFT ][MIDCOL][MIDROW];
        d = cube2[FRONT][MIDCOL][MIDROW];
        rc = SSC(a,b,d);
        if (rc == FAIL)
            return FAIL;

        NICS();
    }

    return OK;
}

/* =====================================================================
SSC
------------------------------------------------------------------------ */

int SSC( uint8_t a, uint8_t b, uint8_t d )
{
    int i;

    IC++;

    for (i = 1; i <= CORNERS; i++)
    {
        if (C[i][1] == a && C[i][2] == b && C[i][3] == d)
        {
            PC[i] = IC;
            return OK;
        }
    }

    return FAIL;
}

/* =====================================================================
SSE
------------------------------------------------------------------------ */

int SSE( uint8_t a, uint8_t b )
{
    int i;

    IE++;

    for (i = 1; i <= EDGES; i++)
    {
        if (E[i][1] == a && E[i][2] == b)
        {
            PE[i] = IE;
            return OK;
        }
    }

    return FAIL;
}

/* =====================================================================
Count Z
------------------------------------------------------------------------ */

void Count_Z( void )
{
    int i,j;

    for (i = 1; i < CORNERS; i++)
    {
        for (j = i+1; j <= CORNERS; j++)
        {
            if (PC[i] > PC[j])
                Z++;
        }
    }

    for (i = 1; i < EDGES; i++)
    {
        for (j = i+1; j <= EDGES; j++)
        {
            if (PE[i] > PE[j])
                Z++;
        }
    }
}

/* =========================== EOF =================================== */
