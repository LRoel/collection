#include "map_db.h"

/** Constructor of class Map
 * 初始化MySQL cpp connector, 设置初始节点数和道路数
 */
Map::Map(const std::string host, const std::string user, const std::string pass, const std::string database)
:driver_(get_driver_instance()), con_(driver_->connect(host, user, pass))
{
    con_->setSchema(database);

    n_vertex_ = getVertexNum();
    n_edge_ = getEdgeNum();
}

int Map::getVertexNum()
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT COUNT(*) FROM vertex"));
    res.reset(pstmt->executeQuery());
    while (res->next())
        n_vertex_ = res->getInt("COUNT(*)");
    return n_vertex_;
}

int Map::addVertex(double x, double y, bool interim )
{
    Vertex v = {};
    if (selectVertex(v, x, y) == 0)
    {
        std::cout << "Map::addVertex(double, double, bool) failed: trying to add existed vertex." << std::endl;
        return v.id;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("INSERT INTO vertex VALUES (?,?,?,?)"));
    n_vertex_ ++;//更新最新数据
    pstmt->setInt(1, n_vertex_);
    pstmt->setDouble(2, x);
    pstmt->setDouble(3, y);
    pstmt->setInt(4,interim);
    pstmt->execute();
    std::cout << "Vertex(" << x << ',' << y << ") Added." << std::endl;
    return n_vertex_;
}

int Map::addVertex(Vertex v, bool interim)
{
    return addVertex(v.x, v.y, interim);
}

int Map::delVertex(int id)
{
    if (id > n_vertex_ || id <= 0)
    {
        std::cout << "Map::delVertex(int) failed: Invalid input argument(s) " << id << std::endl;
        return -1;
    }
    std::auto_ptr< sql::PreparedStatement > pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("DELETE FROM vertex WHERE id=?"));
    pstmt->setInt(1,id);
    res.reset(pstmt->executeQuery());
    while(res->next())
    {
        std::cout <<"Deleting vertex("<<id<<")..." <<std::endl;
        n_vertex_--;
    }
    std::cout <<"Deleting vertex("<<id<<")..." <<std::endl;
    return 0;
}

int Map::selectVertex(Vertex& v, int id)
{
    if (id > n_vertex_ || id <= 0)
    {
        std::cout << "Map::selectVertex(Vertex&, int) failed: Invalid input argument(s) " << id << std::endl;
        return -1;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM vertex WHERE id=?"));
    pstmt->setInt(1,id);
    res.reset(pstmt->executeQuery());
    if(res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << id << ") does not exist. Returning -1." << std::endl;
        return -1;
    }
    while (res->next())
    {
        v.id = id;
        v.x = res->getDouble("x");
        v.y = res->getDouble("y");
        v.interim = res->getBoolean("interim");
    }
    return 0;
}

int Map::selectVertex(Vertex& v, double x, double y, double r)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM vertex WHERE (POW((x-?), 2) + POW((y-?), 2) <= POW(?, 2))"));
    pstmt->setDouble(1, x);
    pstmt->setDouble(2, y);
    pstmt->setDouble(3, r);
    res.reset(pstmt->executeQuery());
    if(res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << x << ',' << y << ") does not exist. Returning -1." << std::endl;
        return -1;
    }
    while (res->next())
    {
        v.id = res->getInt("id");
        v.x = res->getDouble("x");
        v.y = res->getDouble("y");
        v.interim = res->getBoolean("interim");
    }
    return 0;
}

int Map::locateVertex(double x, double y, double r)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;
    Vertex v = {};

    pstmt.reset(con_->prepareStatement("SELECT * FROM vertex WHERE (POW((x-?), 2) + POW((y-?), 2) <= POW(?, 2))"));
    pstmt->setDouble(1, x);
    pstmt->setDouble(2, y);
    pstmt->setDouble(3, r);
    res.reset(pstmt->executeQuery());
    if(res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << x << ',' << y << ") does not exist. Returning -1" << std::endl;
        return -1;
    }
    while (res->next())
    {
        return res->getInt("id");
    }
    return 0;
}

int Map::locateVertex(Vertex v, double r)
{
    return locateVertex(v.x, v.y, r);
}

int Map::getEdgeNum()
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT COUNT(*) FROM edge"));
    res.reset(pstmt->executeQuery());
    while (res->next())
        n_edge_ = res->getInt("COUNT(*)");
    return n_edge_;
}

int Map::addEdge(double length, int v1, int v2, bool interim)
{
    if (v1 == v2 || length <= 0 || v1 > n_vertex_ || v1 <= 0 || v2 > n_vertex_ || v2 <= 0)
    {
        std::cout << "Map::addEdge(double, int, int, bool) failed: Invalid input argument(s)" << v1 << ' ' << v2 << std::endl;
        return -1;
    }
    Edge e = {};
    if (selectEdge(e, v1, v2) == 0)
    {
        std::cout << "Map::addEdge(double, int, int, bool) failed: Trying to add existed edge." << std::endl;
        return e.id;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("INSERT INTO edge VALUES (?,?,?,?,?)"));
    n_edge_++;
    pstmt->setInt(1, n_edge_);
    pstmt->setDouble(2, length);
    pstmt->setInt(3, v1);
    pstmt->setInt(4, v2);
    pstmt->setInt(5, interim);
    pstmt->execute();
    std::cout << "Edge(" << length <<','<< v1 << ',' << v2 << ',' << ") added." <<std::endl;
    return n_edge_;
}

int Map::addEdge(Edge e, bool interim)
{
    return addEdge(e.length, e.v1, e.v2, interim);
}

int Map::delEdge(int id)
{
    if (id > n_edge_ || id <= 0)
    {
        std::cout << "Map::delEdge(int) failed: Invalid input argument(s) " << id << std::endl;
        return -1;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;
    pstmt.reset(con_->prepareStatement("DELETE FROM edge WHERE id=?"));
    pstmt->setInt(1, id);
    res.reset(pstmt->executeQuery());
    while(res->next())
        n_edge_--;
    return 0;
}

int Map::selectEdge(Edge& e, int id)
{
    if (id > n_edge_ || id <= 0)
    {
        std::cout << "Map::selectEdge(Edge&, int) failed: Invalid input argument(s) "<< id << std::endl;
        return -1;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM edge WHERE id=?"));
    pstmt->setInt(1, id);
    res.reset(pstmt->executeQuery());
    if(res->rowsCount() == 0)
    {
        std::cout << "Edge(" << id << ") does not exist. Returning -1." << std::endl;
        return -1;
    }
    while (res->next())
    {
        e.id = res->getInt("id");
        e.length = res->getDouble("length");
        e.v1 = res->getInt("v1");
        e.v2 = res->getInt("v2");
        e.interim = res->getBoolean("interim");
    }
    return 0;
}

int Map::selectEdge(Edge& e, int v1, int v2)
{
    if (v1 == v2 || v1 > n_vertex_ || v1 <= 0 || v2 > n_vertex_ || v2 <= 0)
    {
        std::cout << "Map::selectEdge(Edge&, int, int) failed: Invalid input argument(s) "<< v1 << ' ' << v2 << std::endl;
        return -1;
    }
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM edge WHERE (v1=? AND v2=?) OR (v1=? AND v2=?)"));
    pstmt->setInt(1,v1);
    pstmt->setInt(2,v2);
    pstmt->setInt(3,v2);
    pstmt->setInt(4,v1);
    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Edge(" << v1 << ',' << v2 << ") does not exist. Returning -1." << std::endl;
        return -1;
    }
    while (res->next())
    {
        e.id = res->getInt("id");
        e.length = res->getDouble("length");
        e.v1 = res->getInt("v1");
        e.v2 = res->getInt("v2");
        e.interim = res->getBoolean("interim");
    }
    return 0;
}

int Map::locateEdge(double x, double y, double r)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;
std::cout << "locating vertex("<<x<<','<<y<<")..."<<std::endl;
    Vertex v1 = {}, v2 = {};
    pstmt.reset(con_->prepareStatement("SELECT * FROM edge"));
    res.reset(pstmt->executeQuery());
    if(res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << x << ',' << y << ") is not on any edge." << std::endl;
        return -1;
    }
    int count = 0;
    while (res->next())
    {
        int edge_ID = res->getInt("id");
        std::cout << "###edge_ID = " << edge_ID << "###r = "<<r <<std::endl;
        v1.id = res->getInt("v1");
        v2.id = res->getInt("v2");
        selectVertex(v1, v1.id);
        selectVertex(v2, v2.id);
        // x<-v1 v2<-v1
        double dot_product = (x - v1.x) * (v2.x - v1.x) + (y - v1.y) * (v2.y - v1.y);
        std::cout << "###dot_product = " << dot_product <<std::endl;
        if (dot_product <= 0)
        {
            std::cout << "###"<<sqrt((x - v1.x) * (x - v1.x) + (y - v1.y) * (y - v1.y)) <<std::endl;
            if (sqrt((x - v1.x) * (x - v1.x) + (y - v1.y) * (y - v1.y)) <= r)
            {
                std::cout << "located at "<< edge_ID <<std::endl;
                 return edge_ID;
            }
            else
                continue;
        }
        double seg_sqr = (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y);
        std::cout << "###seg_sqr = " << seg_sqr <<std::endl;
        if (seg_sqr <= dot_product)
        {
            if (sqrt((x - v2.x) * (x - v2.x) + (y - v2.y) * (y - v2.y)) <= r)
            {
                std::cout << "located at "<< edge_ID <<std::endl;
                return edge_ID;
            }
            else
                continue;
        }
        double dr = dot_product / seg_sqr;
        double px = v1.x + (v2.x - v1.x) * dr;
        double py = v1.y + (v2.y - v1.y) * dr;

        if (sqrt((x - px) * (x - px) + (y-py) * (y-py)) <= r)
        {
            std::cout << "located at "<< edge_ID <<std::endl;
             return edge_ID;// 返回路的ID
        }
    }
    std::cout << "located failed"<<std::endl;
    return -1;
}

int Map::locateEdge(Vertex v, double r)
{
    return locateEdge(v.x, v.y, r);
}

double Map::getLength(int id)
{
    Edge e = {};
    selectEdge(e, id);
    return e.length;
}

double Map::getLength(int v1, int v2)
{
    Edge e = {-1};
    selectEdge(e, v1, v2);
    return e.length;
}

int Map::delInterim()
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("DELETE FROM edge WHERE interim=true"));
    pstmt->execute();
    pstmt.reset(con_->prepareStatement("DELETE FROM vertex WHERE interim=true"));
    pstmt->execute();

    pstmt.reset(con_->prepareStatement("SELECT COUNT(*) FROM vertex"));
    res.reset(pstmt->executeQuery());
    while (res->next())
        n_vertex_ = res->getInt("COUNT(*)");
    pstmt.reset(con_->prepareStatement("SELECT COUNT(*) FROM edge"));
    res.reset(pstmt->executeQuery());
    while (res->next())
        n_edge_ = res->getInt("COUNT(*)");
    return 0;
}

int Map::delData()
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("DELETE FROM edge"));
    pstmt->execute();
    pstmt.reset(con_->prepareStatement("DELETE FROM vertex"));
    pstmt->execute();
    n_vertex_ = 0;
    n_edge_ = 0;
    return 0;
}

double Map::distance(Vertex v1, Vertex v2)
{
    return sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
}

double Map::distance(int v1, int v2)
{
    Vertex vertex1 = {}, vertex2 = {};
    selectVertex(vertex1, v1);
    selectVertex(vertex2, v2);
    if (vertex1.id == 0 || vertex2.id == 0)
        return -1;
    return distance(vertex1, vertex2);
}

bool Map::isPhotoSite(int id, int* angle)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM photo_site WHERE vid=?"));
    pstmt->setInt(1,id);

    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << id << ") is not a photo site. Returning -1." << std::endl;
        return false;
    }
    if (angle != NULL)
        while (res->next())
            *angle = res->getInt("angle");
    return true;
}

bool Map::isIntroSite(int id, int* intro_id)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM intro_site WHERE vid=?"));
    pstmt->setInt(1, id);

    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << id << ") is not an intro site. Returning -1." << std::endl;
        return false;
    }
    if (intro_id != NULL)
        while (res->next())
            *intro_id = res->getInt("intro_id");
    return true;
}

bool Map::isCrossing(int id)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM crossing WHERE vid=?"));
    pstmt->setInt(1,id);

    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Vertex(" << id << ") is not a crossing. Returning -1." << std::endl;
        return false;
    }
    return true;
}

std::string Map::selectPatrolPath(int id)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;
std::cout << "SELECT * FROM patrol_route WHERE id=" << id << std::endl;
    pstmt.reset(con_->prepareStatement("SELECT * FROM patrol_route WHERE id=?"));
    pstmt->setInt(1,id);

    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Route(" << id << ") does not exist. " << std::endl;
        return "";
    }
    std::istream* route;
    std::string s;
    while (res->next())
        std::getline(*(res->getBlob("route")), s);
    std::cout << "######## "<< s << std::endl;
    return s;
}

int Map::selectStation(Vertex& v, int id)
{
    std::auto_ptr< sql::PreparedStatement >  pstmt;
    std::auto_ptr< sql::ResultSet > res;

    pstmt.reset(con_->prepareStatement("SELECT * FROM station WHERE id=?"));
    pstmt->setInt(1,id);

    res.reset(pstmt->executeQuery());
    if (res->rowsCount() == 0)
    {
        std::cout << "Station(" << id << ") does not exist. " << std::endl;
        return -1;
    }
    int vid;
    while (res->next())
        vid = res->getInt("vid");
    selectVertex(v, vid);
    return 0;
}
