class RRTPoint
{
private:
    int x, y;
    std::vector<RRTPoint*> connections;
public:
    RRTPoint(int x, int y);
    ~RRTPoint();
};
