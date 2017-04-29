#ifndef MAP_H_
#define MAP_H_

class map
{
public:
	//definition of single landmark:
	struct single_landmark_s
	{
		int id_i ;
		float x_f;
	};

	//list of landmarks:
	std::vector<single_landmark_s> landmark_list ;
};
#endif /* MAP_H_ */
