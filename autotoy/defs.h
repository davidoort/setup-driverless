struct Point{
    float x, y;
};

struct Cones{
    Point points [];
    enum col {yellow, blue, orange}; //yellow: right, blue: left, orange: start end
};

struct Line{
    Point points [];
};

struct Track{
    Point points[];
};