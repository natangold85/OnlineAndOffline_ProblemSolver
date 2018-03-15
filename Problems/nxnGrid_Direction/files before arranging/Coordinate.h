#ifndef COORDINATE_H
#define COORDINATE_H

#include <vector>

class Coordinate
{
public:
	Coordinate() = default;
	Coordinate(int x, int y);
	Coordinate(std::pair<int, int> coord);
	~Coordinate() = default;
	
	Coordinate &operator=(const Coordinate & b) = default;
	Coordinate &operator=(std::pair<int, int> coord);

	friend Coordinate Min(Coordinate & a, Coordinate & b);
	friend Coordinate Min(std::vector<Coordinate> & location, int idx);
	friend Coordinate Max(Coordinate & a, Coordinate & b);
	friend Coordinate Max(std::vector<Coordinate> & location, int idx);

	void Abs();
	Coordinate Sign();

	int &X() { return m_x; };
	int &Y() { return m_y; };

	int X() const { return m_x; };
	int Y() const { return m_y; };
	/// translate (x,y) to idx given a grid size
	int GetIdx(int gridSize) const { return m_x + m_y * gridSize; };
	void SetIdx(int idx, int gridSize) { m_x = idx % gridSize; m_y = idx / gridSize;};
	void Zero();

	Coordinate& operator-=(const Coordinate toDecrease);
	Coordinate& operator+=(const Coordinate toIncrease);
	Coordinate& operator/=(double toDivide);
	Coordinate& operator*=(double toMultiply);

	bool operator>(const Coordinate & b) const;
	bool operator<(const Coordinate & b) const;
	bool operator==(const Coordinate & a) const;
	bool operator!=(const Coordinate & a) const;

	Coordinate operator-(Coordinate & toDecrease);
	Coordinate operator+(Coordinate & toIncrease);
	Coordinate operator/(double toDivide);

	int Distance(const Coordinate &) const;
	double RealDistance(const Coordinate &) const;

	bool ValidLocation(int gridSize) const;
private:
	int m_x;
	int m_y;
};

# endif //COORDINATE_H