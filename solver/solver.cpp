/*
 * Robik - Rubik's cube solver from junk
 *
 * solver.cpp
 *
 * Solving algorithm from http://tomas.rokicki.com/cubecontest/winners.html,
 * Jaap Scherphuis' algorithm
 *
 * Pruning table generation on-the-fly was too slow on LPC2366, so pre-generated
 * tables are used, defined in tables.h. This code only does the recursive
 * tree search, which happens fast enough.
 *
 *  2.9.2017
 */

/*
  Jaap Scherphuis,  24/01/2004,  jaapsch_at_yahoo_do_com

  Thistlethwaite's algorithm.
*/


#include <string.h>
#include <cr_section_macros.h>

#include "solver.h"
#include "tinyprintf.h"

#include "tables.h"

//using namespace std ;
char
	// RLFBUD is the face order used for input, so that a correctly oriented
	// piece in the input has its 'highest value' facelet first. The rest of the
	// program uses moves in FBRLUD order.
	*faces=(char *)"RLFBUD",
	// I use char arrays here cause they can be initialised with a string
	// which is shorter than initialising other arrays.
	// Internally cube uses slightly different ordering to the input so that
	//  orbits of stage 4 are contiguous. Note also that the two corner orbits
	//  are diametrically opposite each other.
	//input:  UF UR UB UL  DF DR DB DL  FR FL BR BL  UFR URB UBL ULF   DRF DFL DLB DBR
	//        A  B  C  D   E  F  G  H   I  J  K  L   M   N   O   P     Q   R   S   T
	//        A  E  C  G   B  F  D  H   I  J  K  L   M   S   N   T     R   O   Q   P
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	*order=(char *)"AECGBFDHIJKLMSNTROQP",
	//To quickly recognise the pieces, I construct an integer by setting a bit for each
	// facelet. The unique result is then found on the list below to map it to the correct
	// cubelet of the cube.
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	//bithash:20,36,24,40, 17,33,18,34, 5, 6, 9, 10, 21, 26, 38, 41,   42, 37, 25, 22
	*bithash=(char *)"TdXhQaRbEFIJUZfijeYV",
	//Each move consists of two 4-cycles. This string contains these in FBRLUD order.
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	//        A  B  C  D   E  F  G  H   I  J  K  L   M   N   O   P     Q   R   S   T
	*perm=(char *)"AIBJTMROCLDKSNQPEKFIMSPRGJHLNTOQAGCEMTNSBFDHORPQ",

	// current cube position
	pos[20],ori[20],val[20],
	// temporary variable used in swap macro
	TEMP;
	// pruning tables, 2 for each phase
	// *** modification: use pre-generated tables, defined in tables.h
	char *tables[8] = { table0,table1,table2,table3,table4,table5,table6,table7 };
	// current phase solution
int move[20],moveamount[20],
	// current phase being searched (0,2,4,6 for phases 1 to 4)
	phase=0,
	// Length of pruning tables. (one dummy in phase 1);
	tablesize[]={1,4096,  6561,4096,  256,1536,  13824,576};

// Use very ugly and unsafe macro to swap items instead of classic routine with
//   pointers for the sole reason of brevity
#define SWAP(a,b) TEMP=a;a=b;b=TEMP;
// number 65='A' is often subtracted to convert char ABC... to number 0,1,2,...
#define CHAROFFSET 65

// *** modification: SendMove() stuff

#define EMPTY           'X'
#define END_OF_MOVES    'Z'

char prev_face = EMPTY;
int prev_turns = 0;

void SendMove( char face, int turns );

// *** end of modification

// Cycles 4 pieces in array p, the piece indices given by a[0..3].
void cycle(char*p,char*a){
	SWAP(p[*a-CHAROFFSET],p[a[1]-CHAROFFSET]);
	SWAP(p[*a-CHAROFFSET],p[a[2]-CHAROFFSET]);
	SWAP(p[*a-CHAROFFSET],p[a[3]-CHAROFFSET]);
}

// twists i-th piece a+1 times.
void twist(int i,int a){
	i-=CHAROFFSET;
	ori[i]=(ori[i]+a+1)%val[i];
}


// set cube to solved position
void reset(){
	for( int i=0; i<20; pos[i]=i, ori[i++]=0);
}

// convert permutation of 4 chars to a number in range 0..23
int permtonum(char* p){
	int n=0;
	for ( int a=0; a<4; a++) {
		n*=4-a;
		for( int b=a; ++b<4; )
			if (p[b]<p[a]) n++;
	}
	return n;
}

// convert number in range 0..23 to permutation of 4 chars.
void numtoperm(char* p,int n,int o){
	p+=o;
	p[3]=o;
	for (int a=3; a--;){
		p[a] = n%(4-a) +o;
		n/=4-a;
		for (int b=a; ++b<4; )
			if ( p[b] >= p[a]) p[b]++;
	}
}

// get index of cube position from table t
int getposition(int t){
	int i=-1,n=0;
	switch(t){
	// case 0 does nothing so returns 0
	case 1://edgeflip
		// 12 bits, set bit if edge is flipped
		for(;++i<12;) n+= ori[i]<<i;
		break;
	case 2://cornertwist
		// get base 3 number of 8 digits - each digit is corner twist
		for(i=20;--i>11;) n=n*3+ori[i];
		break;
	case 3://middle edge choice
		// 12 bits, set bit if edge belongs in Um middle slice
		for(;++i<12;) n+= (pos[i]&8)?(1<<i):0;
		break;
	case 4://ud slice choice
		// 8 bits, set bit if UD edge belongs in Fm middle slice
		for(;++i<8;) n+= (pos[i]&4)?(1<<i):0;
		break;
	case 5://tetrad choice, twist and parity
		int corn[8],j,k,l,corn2[4];
		// 8 bits, set bit if corner belongs in second tetrad.
		// also separate pieces for twist/parity determination
		k=j=0;
		for(;++i<8;)
			if((l=pos[i+12]-12)&4){
				corn[l]=k++;
				n+=1<<i;
			}else corn[j++]=l;
		//Find permutation of second tetrad after solving first
		for(i=0;i<4;i++) corn2[i]=corn[4+corn[i]];
		//Solve one piece of second tetrad
		for(;--i;) corn2[i]^=corn2[0];

		// encode parity/tetrad twist
		n=n*6+corn2[1]*2-2;
		if(corn2[3]<corn2[2])n++;
		break;
	case 6://two edge and one corner orbit, permutation
		n=permtonum(pos)*576+permtonum(pos+4)*24+permtonum(pos+12);
		break;
	case 7://one edge and one corner orbit, permutation
		n=permtonum(pos+8)*24+permtonum(pos+16);
		break;
	}
	return n;
}


// sets cube to any position which has index n in table t
void setposition(int t, int n){
	int i=0,j=12,k=0;
	char *corn=(char *)"QRSTQRTSQSRTQTRSQSTRQTSR";
	reset();
	switch(t){
	// case 0 does nothing so leaves cube solved
	case 1://edgeflip
		for(;i<12;i++,n>>=1) ori[i]=n&1;
		break;
	case 2://cornertwist
		for(i=12;i<20;i++,n/=3) ori[i]=n%3;
		break;
	case 3://middle edge choice
		for(;i<12;i++,n>>=1) pos[i]= 8*n&8;
		break;
	case 4://ud slice choice
		for(;i<8;i++,n>>=1) pos[i]= 4*n&4;
		break;
	case 5://tetrad choice,parity,twist
		corn+=n%6*4;
		n/=6;
		for(;i<8;i++,n>>=1)
			pos[i+12]= n&1 ? corn[k++]-CHAROFFSET : j++;
		break;
	case 6://slice permutations
		numtoperm(pos,n%24,12);n/=24;
		numtoperm(pos,n%24,4); n/=24;
		numtoperm(pos,n   ,0);
		break;
	case 7://corner permutations
		numtoperm(pos,n/24,8);
		numtoperm(pos,n%24,16);
		break;
	}
}


//do a clockwise quarter turn cube move
void domove(int m){
	char *p=perm+8*m;
    int i=8;
	//cycle the edges
	cycle(pos,p);
	cycle(ori,p);
	//cycle the corners
	cycle(pos,p+4);
	cycle(ori,p+4);
	//twist corners if RLFB
	if(m<4)
		for(;--i>3;) twist(p[i],i&1);
	//flip edges if FB
	if(m<2)
		for(i=4;i--;) twist(p[i],0);
}

#if(0)  // --------- table calculation removed -----------
// calculate a pruning table
void filltable(int ti){
	int n=1,l=1, tl=tablesize[ti];
	// alocate table memory
	char* tb = tables[ti]=new char[tl];
	//clear table
	memset( tb, 0, tl);
	//mark solved position as depth 1
	reset();
	tb[getposition(ti)]=1;

	// while there are positions of depth l
	while(n){
		n=0;
		// find each position of depth l
		for(int i=0;i<tl;i++){
			if( tb[i]==l ){
				//construct that cube position
				setposition(ti,i);
				// try each face any amount
				for( int f=0; f<6; f++){
					for( int q=1;q<4;q++){
						domove(f);
						// get resulting position
						int r=getposition(ti);
						// if move as allowed in that phase, and position is a new one
						if( ( q==2 || f>=(ti&6) ) && !tb[r]){
							// mark that position as depth l+1
							tb[r]=l+1;
							n++;
						}
					}
					domove(f);
				}
			}
		}
		l++;
	}
}
#endif  // ------ end of removed calculation --------------

// Pruned tree search. recursive.
bool searchphase(int movesleft, int movesdone,int lastmove){
	// prune - position must still be solvable in the remaining moves available
	if( tables[phase  ][getposition(phase  )]-1 > movesleft ||
	    tables[phase+1][getposition(phase+1)]-1 > movesleft ) return false;

	// If no moves left to do, we have solved this phase
	if(!movesleft) return true;

	// not solved. try each face move
	for( int i=6;i--;){
		// do not repeat same face, nor do opposite after DLB.
		if( i-lastmove && (i-lastmove+1 || i|1 ) ){
			move[movesdone]=i;
			// try 1,2,3 quarter turns of that face
			for(int j=0;++j<4;){
				//do move and remember it
				domove(i);
				moveamount[movesdone]=j;
				//Check if phase only allows half moves of this face
				if( (j==2 || i>=phase ) &&
					//search on
					searchphase(movesleft-1,movesdone+1,i) ) return true;
			}
			// put face back to original position.
			domove(i);
		}
	}
	// no solution found
	return false;
}

void solver(char intbl[20][4])
{
	int f,i=0,j=0,k=0,pc,mor;

	for(; k<20; k++)
		val[k]=k<12?2:3;

	// read input, 20 pieces worth
	for(; i<20; i++){
		f=pc=k=mor=0;
		for(;f<val[i];f++){
			// modification: take input from intbl[]
			j=strchr(faces,intbl[i][f])-faces;
			// keep track of principal facelet for orientation
			if(j>k) {k=j;mor=f;}
			//construct bit hash code
			pc+= 1<<j;
		}
		// find which cubelet it belongs, i.e. the label for this piece
		for(f=0; f<20; f++)
			if( pc==bithash[f]-64 ) break;
		// store piece
		pos[order[i]-CHAROFFSET]=f;
		ori[order[i]-CHAROFFSET]=mor%val[i];
	}

	//solve the cube
	// four phases
	for( phase=0; phase<8; phase+=2){
		// try each depth till solved
		for( j=0; !searchphase(j,0,9); j++);
		//output result of this phase
		for( i=0; i<j; i++)
			SendMove("FBRLUD"[move[i]],moveamount[i]);
			//printf((char *)"%c%d","FBRLUD"[move[i]],moveamount[i]);
	}

    SendMove(END_OF_MOVES,0);
    //printf((char *)"\r");
}

/* =====================================================================
Solver algorithm sometimes produces consequent same face moves. This
function optimizes them out before sending to main board.
--------------------------------------------------------------------- */

void SendMove( char face, int turns )
{
    if (face == END_OF_MOVES)
    {
        // end of moves: send buffered move and end marker
        if (prev_face != EMPTY)
            printf((char *)"%c%d",prev_face,prev_turns);

        printf((char *)"\r");

        prev_face = EMPTY;
        return;
    }

    if (prev_face == EMPTY)
    {
        // buffer empty: just buffer the move
        prev_face = face;
        prev_turns = turns;
    }
    else if (face == prev_face)
    {
        // same face as before: combine the turns
        turns = (turns + prev_turns) & 0x03;

        if (turns != 0)
            printf((char *)"%c%d",face,turns);

        prev_face = EMPTY;
    }
    else
    {
        // normal case: send buffered move, buffer the new one
        printf((char *)"%c%d",prev_face,prev_turns);

        prev_face = face;
        prev_turns = turns;
    }
}

/* ============================ EOF ====================================== */

