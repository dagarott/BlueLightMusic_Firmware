#ifndef SONGPATTERNS_H__
#define SONGPATTERNS_H__


//define timer scaling value for each note
#define C 2441
#define D 2741
#define E 3048
#define F 3255
#define G 3654
#define A 4058
#define B 4562
#define C2 4882

#define num_notes_song1 14 //total number of notes in song to be played - modify for specific song
#define tempo_song1 100
int song1[x]={C, C, G, G, A, A, G, F, F, E, E, D, D, C}; //insert notes of song in array
int length_song1[x]={1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2}; //relative length of each note

#define num_notes_song2 26
#define tempo_song2 250
int song2[x] = {B, A, G, A, B, B, B, A, A, A, B, B, B, B, A, G, A, B, B, B, A, A, B, A, G, G};
int length_song2[x] = {1, 1, 1, 1, 1, 1, 2, 1, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 2};
	
#define num_notes_song3 25
#define tempo_song3 400
int song3[x] = {E, E, E, E, E, E, E, G, C, D, E, F, F, F, F, F, E, E, E, E, D, D, E, D, G};
int length_song3[x] = {1, 1, 2, 1, 1, 2, 1, 1, 1, 1, 4, 1

#endif // SONGPATTERNS_H