#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

__global__ void translatory_motion(double *arr_position, double *new_position, double *arr_speed,
                                   int size)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = idx; i < size; i += stride) {

    new_position[i] += arr_position[i] + arr_speed[i];
  }
}

class RunTimeCalculator {
public:
  void start()
  {
    start_time = std::chrono::high_resolution_clock::now();
  }
  double getTime()
  {
    auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(
                          std::chrono::high_resolution_clock::now() - start_time)
                          .count();
    return elapsed_time;
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

class RandomNumberGenerator {
public:
  RandomNumberGenerator()
  {
  }

  static inline double getRandom(double begin_range, double end_range)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(begin_range, end_range);

    return dis(gen);
  }
};

class Sport {
private:
  unsigned int distance;

public:
  Sport(unsigned int _distance) : distance{ _distance }
  {
  }
  unsigned int getDistance() const
  {
    return distance;
  }
};

class Swimming : public Sport {
public:
  Swimming(unsigned int _distance) : Sport{ _distance }
  {
  }
};

class Cycling : public Sport {
public:
  Cycling(unsigned int _distance) : Sport{ _distance }
  {
  }
};

class Run : public Sport {
public:
  Run(unsigned int _distance) : Sport{ _distance }
  {
  }
};

class Triathlon {
private:
  std::vector<Sport> sport_set;

public:
  Triathlon(unsigned int swim_lane_length, unsigned int velodrom_length, unsigned int runway_length)
  {
    sport_set.push_back(Swimming(swim_lane_length));
    sport_set.push_back(Cycling(velodrom_length));
    sport_set.push_back(Run(runway_length));
  }

  void print_distance()
  {
    for (const auto &sport : sport_set) {
      std::cout << sport.getDistance() << "\n";
    }
  }

  const std::vector<Sport> getStages()
  {
    return sport_set;
  }
};

struct CompleteRace {
  std::string name;
  int team_name;
  double total_time;
};

class Sporter {
private:
  std::string name;

public:
  Sporter(std::string _name) : name(_name)
  {
  }
  const std::string getName() const
  {
    return name;
  }
};

class Competitor : public Sporter {
private:
  bool complete;
  int stage;
  int standby_time;
  int team_name;
  double prev_position;
  double position;
  double speed;
  double total_time;

public:
  Competitor(int _team_name, std::string _name, double _speed)
      : Sporter(_name), speed(_speed), team_name(_team_name)
  {
    position = 0.0;
    prev_position = 0.0;
    stage = 0;
    complete = false;
    standby_time = 0;
    total_time = 0.0;
  }

  double getPosition() const
  {
    return position;
  }
  int getTeamName() const
  {
    return team_name;
  }
  void setPosition(double new_position)
  {
    prev_position = position;
    position = new_position;
  }

  double getPrevPosition() const
  {
    return prev_position;
  }

  double getSpeed() const
  {
    return speed;
  }

  void setSpeed(double new_speed)
  {
    speed = new_speed;
  }

  int getStage() const
  {
    return stage;
  }

  void increaseStage()
  {
    stage++;
  }

  bool getComplete() const
  {
    return complete;
  }

  void setComplete()
  {
    complete = true;
  }
  void decreaseStandbyTime()
  {
    standby_time--;
  }
  int getStandbyTime() const
  {
    return standby_time;
  }
  void setStandbyTime(int a)
  {
    standby_time = a;
  }
  double getTotalTime() const
  {
    return total_time;
  }
  void increaseTotalTime()
  {
    total_time += 1.0;
  }
  void setTotalTime(double new_total_time)
  {
    total_time = new_total_time;
  }
};

class TriathlonTeam {
private:
  std::array<Competitor, 3> team;

public:
  TriathlonTeam(Competitor c1, Competitor c2, Competitor c3) : team{ c1, c2, c3 } {};

  std::array<Competitor, 3> getTeam() const
  {
    return team;
  }

  bool setTeamNewStats(std::vector<double> vec, int team_name, Triathlon &t,
                       std::vector<CompleteRace> &sorted)
  {
    bool first_print = false;
    for (size_t i = 0; i < 3; i++) {
      if (!team[i].getComplete()) {
        if ((team[i].getStage() == 0 && vec[i] >= t.getStages()[0].getDistance())) {
          team[i].increaseStage();
          team[i].setStandbyTime(10);
          team[i].setSpeed(team[i].getSpeed() * 3);
        } else if (team[i].getStage() == 1
                   && vec[i] >= t.getStages()[0].getDistance() + t.getStages()[1].getDistance()) {
          team[i].increaseStage();
          team[i].setStandbyTime(10);
          team[i].setSpeed(team[i].getSpeed() / 9);
        } else if (team[i].getStage() == 2
                   && vec[i] >= t.getStages()[0].getDistance() + t.getStages()[1].getDistance()
                                  + t.getStages()[2].getDistance()) {
          team[i].increaseStage();
          team[i].setComplete();
          if (sorted.size() == 0) {
            first_print = true;
          }
          sorted.push_back({ team[i].getName(), team_name, team[i].getTotalTime() });
        }
        if (team[i].getStandbyTime() != 0) {
          team[i].decreaseStandbyTime();
        } else {
          team[i].setPosition(vec[i]);
        }
        team[i].increaseTotalTime();
      }
    }
    return first_print;
  }
};

bool compareTeamName(const CompleteRace &first, const CompleteRace &second)
{
  return first.team_name < second.team_name;
}
bool compareTotalTime(const CompleteRace &first, const CompleteRace &second)
{
  return first.total_time < second.total_time;
}

void runTimeGetCompetitor(std::vector<TriathlonTeam> &teams)
{
  while (true) {
    std::string input;
    std::cin >> input;
    std::vector<std::string> seperated_input;

    auto split_string = [&seperated_input](const std::string &str) {
      std::istringstream iss(str);
      std::string token;

      while (iss >> std::quoted(token)) {
        seperated_input.push_back(token);
      }
    };

    split_string(input);
    std::cout << "Requested Inputs:\n";
    for (const auto &team : teams) {
      for (const auto &competitor : team.getTeam()) {
        for (const auto &i : seperated_input) {
          if (i == competitor.getName()) {
            std::cout << " Team Name: " << competitor.getTeamName()
                      << " Name: " << competitor.getName()
                      << " Position: " << competitor.getPosition()
                      << " Speed: " << competitor.getSpeed() << "\n";
          }
        }
      }
    }
  }
}

class Race {
private:
  std::vector<TriathlonTeam> teams;
  Triathlon stages;
  std::vector<CompleteRace> sorted;

public:
  Race(unsigned int stage1, unsigned int stage2, unsigned int stage3)
      : stages{ stage1, stage2, stage3 }
  {
  }
  void InitRace(const std::vector<std::string> sporter_list, double speed_range_begin,
                double speed_range_end)
  {
    if (sporter_list.size()) {
      if (sporter_list.size() % 3 != 0) {
        throw std::runtime_error("Missing team member.");
      }
      teams.reserve(sporter_list.size() / 3);
    } else {
      throw std::runtime_error("No team members.");
    }
    int team_count = 0;
    for (size_t i = 0; i < sporter_list.size(); i += 3) {

      teams.emplace_back(TriathlonTeam{
        Competitor{ team_count, sporter_list[i],
                    RandomNumberGenerator::getRandom(speed_range_begin, speed_range_end) },
        Competitor{ team_count, sporter_list[i + 1],
                    RandomNumberGenerator::getRandom(speed_range_begin, speed_range_end) },
        Competitor{ team_count, sporter_list[i + 2],
                    RandomNumberGenerator::getRandom(speed_range_begin, speed_range_end) } });
      team_count++;
    }
  }

  void RaceStart()
  {
    std::thread input_t([this]() { runTimeGetCompetitor(teams); });

    double *d_position;
    double *d_new_position;
    double *d_speed;

    size_t COMPETITOR_SIZE = teams.size() * teams[0].getTeam().size();

    cudaMalloc((void **)&d_position, COMPETITOR_SIZE * sizeof(double));
    cudaMalloc((void **)&d_new_position, COMPETITOR_SIZE * sizeof(double));
    cudaMalloc((void **)&d_speed, COMPETITOR_SIZE * sizeof(double));
    while (true) {

      if (sorted.size() == teams.size() * teams[0].getTeam().size()) {
        input_t.detach();
        break;
      }
      std::vector<double> temp_competitor_position;
      std::vector<double> temp_competitor_new_position(COMPETITOR_SIZE, 0.0);
      std::vector<double> temp_competitor_speed;

      for (const auto &team : teams) {
        for (const auto &competitor : team.getTeam()) {
          temp_competitor_position.push_back(competitor.getPosition());
          temp_competitor_speed.push_back(competitor.getSpeed());
        }
      }

      cudaMemcpy(d_position, temp_competitor_position.data(), COMPETITOR_SIZE * sizeof(double),
                 cudaMemcpyHostToDevice);

      cudaMemcpy(d_new_position, temp_competitor_new_position.data(),
                 COMPETITOR_SIZE * sizeof(double), cudaMemcpyHostToDevice);

      cudaMemcpy(d_speed, temp_competitor_speed.data(), COMPETITOR_SIZE * sizeof(double),
                 cudaMemcpyHostToDevice);

      int block_size = 256;
      int num_blocks = (COMPETITOR_SIZE + block_size - 1) / block_size;

      translatory_motion<<<num_blocks, block_size>>>(d_position, d_new_position, d_speed,
                                                     COMPETITOR_SIZE);
      cudaDeviceSynchronize();

      cudaMemcpy(temp_competitor_position.data(), d_position, COMPETITOR_SIZE * sizeof(double),
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(temp_competitor_new_position.data(), d_new_position,
                 COMPETITOR_SIZE * sizeof(double), cudaMemcpyDeviceToHost);
      cudaMemcpy(temp_competitor_speed.data(), d_speed, COMPETITOR_SIZE * sizeof(double),
                 cudaMemcpyDeviceToHost);

      divideOfThree(temp_competitor_new_position);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    cudaFree(d_position);
    cudaFree(d_new_position);
    cudaFree(d_speed);

    printResult();
  }

  void printStats()
  {
    std::cout << std::setw(13) << std::left << "Name" << std::setw(13) << std::left << "Speed"
              << std::setw(13) << std::left << "Position" << std::setw(13) << std::left
              << "PrevPosition" << std::setw(13) << std::left << "Stage" << std::setw(13)
              << std::left << "Standby Time" << std::setw(13) << std::left << "Complete"
              << std::setw(13) << std::left << "Total Time"
              << "\n";
    for (const auto &team : teams) {
      for (const auto &s : team.getTeam()) {
        std::cout << std::setw(13) << std::left << s.getName() << std::setw(13) << std::left
                  << s.getSpeed() << std::setw(13) << std::left << s.getPosition() << std::setw(13)
                  << std::left << s.getPrevPosition() << std::setw(13) << std::left << s.getStage()
                  << std::setw(13) << std::left << s.getStandbyTime() << std::setw(13) << std::left
                  << s.getComplete() << std::setw(13) << std::left << s.getTotalTime() << "\n";
      }
    }
  }
  void printResult()
  {
    std::cout << "\n"
              << "-------------- RESULT --------------\n";
    std::sort(sorted.begin(), sorted.end(), compareTotalTime);
    std::cout << "BEST SPORTER => " << sorted[0].name << " | Team Name: " << sorted[0].team_name
              << " | Total Time: " << sorted[0].total_time << "\n";
    std::sort(sorted.begin(), sorted.end(), compareTeamName);
    std::pair<int, double> best{ std::make_pair<int, double>(-1, 0.0) };

    for (size_t i = 0; i < sorted.size(); i += 3) {
      double sum = sorted[i].total_time + sorted[i + 1].total_time + sorted[i + 2].total_time;
      if (best.second == 0.0) {
        best.first = 0;
        best.second = sum;
      }
      if (sum < best.second) {
        best.first = sorted[i].team_name, best.second = sum;
      }
    }
    std::cout << "BEST TEAM  => "
              << "Team Name: " << best.first << " | Total Time: " << best.second << "\n";
  }

private:
  void divideOfThree(std::vector<double> &temp_new_position)
  {
    auto it = temp_new_position.begin();
    size_t team_count = 0;
    while (it != temp_new_position.end()) {
      std::vector<double> group;
      group.reserve(3);

      for (int i = 0; i < 3 && it != temp_new_position.end(); ++i, ++it) {
        group.push_back(*it);
      }

      if (teams[team_count].setTeamNewStats(group, team_count, stages, sorted)) {
        std::cout << "Firt to Finish =>  Name: " << sorted[0].name
                  << " | Team Name: " << sorted[0].team_name
                  << " | Total Time: " << sorted[0].total_time << "\n\n";
      }
      team_count++;
      if (sorted.size() == teams.size() * teams[0].getTeam().size()) {
        printStats();
      }
    }
  }
};

int main()
{
  std::vector<std::string> sporter_name{
    "s1",   "s2",   "s3",   "s4",   "s5",   "s6",   "s7",   "s8",   "s9",   "s10",  "s11",  "s12",
    "s13",  "s14",  "s15",  "s16",  "s17",  "s18",  "s19",  "s20",  "s21",  "s22",  "s23",  "s24",
    "s25",  "s26",  "s27",  "s28",  "s29",  "s30",  "s31",  "s32",  "s33",  "s34",  "s35",  "s36",
    "s37",  "s38",  "s39",  "s40",  "s41",  "s42",  "s43",  "s44",  "s45",  "s46",  "s47",  "s48",
    "s49",  "s50",  "s51",  "s52",  "s53",  "s54",  "s55",  "s56",  "s57",  "s58",  "s59",  "s60",
    "s61",  "s62",  "s63",  "s64",  "s65",  "s66",  "s67",  "s68",  "s69",  "s70",  "s71",  "s72",
    "s73",  "s74",  "s75",  "s76",  "s77",  "s78",  "s79",  "s80",  "s81",  "s82",  "s83",  "s84",
    "s85",  "s86",  "s87",  "s88",  "s89",  "s90",  "s91",  "s92",  "s93",  "s94",  "s95",  "s96",
    "s97",  "s98",  "s99",  "s100", "s101", "s102", "s103", "s104", "s105", "s106", "s107", "s108",
    "s109", "s110", "s111", "s112", "s113", "s114", "s115", "s116", "s117", "s118", "s119", "s120",
    "s121", "s122", "s123", "s124", "s125", "s126", "s127", "s128", "s129", "s130", "s131", "s132",
    "s133", "s134", "s135", "s136", "s137", "s138", "s139", "s140", "s141", "s142", "s143", "s144",
    "s145", "s146", "s147", "s148", "s149", "s150", "s151", "s152", "s153", "s154", "s155", "s156",
    "s157", "s158", "s159", "s160", "s161", "s162", "s163", "s164", "s165", "s166", "s167", "s168",
    "s169", "s170", "s171", "s172", "s173", "s174", "s175", "s176", "s177", "s178", "s179", "s180",
    "s181", "s182", "s183", "s184", "s185", "s186", "s187", "s188", "s189", "s190", "s191", "s192",
    "s193", "s194", "s195", "s196", "s197", "s198", "s199", "s200", "s201", "s202", "s203", "s204",
    "s205", "s206", "s207", "s208", "s209", "s210", "s211", "s212", "s213", "s214", "s215", "s216",
    "s217", "s218", "s219", "s220", "s221", "s222", "s223", "s224", "s225", "s226", "s227", "s228",
    "s229", "s230", "s231", "s232", "s233", "s234", "s235", "s236", "s237", "s238", "s239", "s240",
    "s241", "s242", "s243", "s244", "s245", "s246", "s247", "s248", "s249", "s250", "s251", "s252",
    "s253", "s254", "s255", "s256", "s257", "s258", "s259", "s260", "s261", "s262", "s263", "s264",
    "s265", "s266", "s267", "s268", "s269", "s270", "s271", "s272", "s273", "s274", "s275", "s276",
    "s277", "s278", "s279", "s280", "s281", "s282", "s283", "s284", "s285", "s286", "s287", "s288",
    "s289", "s290", "s291", "s292", "s293", "s294", "s295", "s296", "s297", "s298", "s299", "s300",
    "s301", "s302", "s303", "s304", "s305", "s306", "s307", "s308", "s309", "s310", "s311", "s312",
    "s313", "s314", "s315", "s316", "s317", "s318", "s319", "s320", "s321", "s322", "s323", "s324",
    "s325", "s326", "s327", "s328", "s329", "s330", "s331", "s332", "s333", "s334", "s335", "s336",
    "s337", "s338", "s339", "s340", "s341", "s342", "s343", "s344", "s345", "s346", "s347", "s348",
    "s349", "s350", "s351", "s352", "s353", "s354", "s355", "s356", "s357", "s358", "s359", "s360",
    "s361", "s362", "s363", "s364", "s365", "s366", "s367", "s368", "s369", "s370", "s371", "s372",
    "s373", "s374", "s375", "s376", "s377", "s378", "s379", "s380", "s381", "s382", "s383", "s384",
    "s385", "s386", "s387", "s388", "s389", "s390", "s391", "s392", "s393", "s394", "s395", "s396",
    "s397", "s398", "s399", "s400", "s401", "s402", "s403", "s404", "s405", "s406", "s407", "s408",
    "s409", "s410", "s411", "s412", "s413", "s414", "s415", "s416", "s417", "s418", "s419", "s420",
    "s421", "s422", "s423", "s424", "s425", "s426", "s427", "s428", "s429", "s430", "s431", "s432",
    "s433", "s434", "s435", "s436", "s437", "s438", "s439", "s440", "s441", "s442", "s443", "s444",
    "s445", "s446", "s447", "s448", "s449", "s450", "s451", "s452", "s453", "s454", "s455", "s456",
    "s457", "s458", "s459", "s460", "s461", "s462", "s463", "s464", "s465", "s466", "s467", "s468",
    "s469", "s470", "s471", "s472", "s473", "s474", "s475", "s476", "s477", "s478", "s479", "s480",
    "s481", "s482", "s483", "s484", "s485", "s486", "s487", "s488", "s489", "s490", "s491", "s492",
    "s493", "s494", "s495", "s496", "s497", "s498", "s499", "s500", "s501", "s502", "s503", "s504",
    "s505", "s506", "s507", "s508", "s509", "s510", "s511", "s512", "s513", "s514", "s515", "s516",
    "s517", "s518", "s519", "s520", "s521", "s522", "s523", "s524", "s525", "s526", "s527", "s528",
    "s529", "s530", "s531", "s532", "s533", "s534", "s535", "s536", "s537", "s538", "s539", "s540",
    "s541", "s542", "s543", "s544", "s545", "s546", "s547", "s548", "s549", "s550", "s551", "s552",
    "s553", "s554", "s555", "s556", "s557", "s558", "s559", "s560", "s561", "s562", "s563", "s564",
    "s565", "s566", "s567", "s568", "s569", "s570", "s571", "s572", "s573", "s574", "s575", "s576",
    "s577", "s578", "s579", "s580", "s581", "s582", "s583", "s584", "s585", "s586", "s587", "s588",
    "s589", "s590", "s591", "s592", "s593", "s594", "s595", "s596", "s597", "s598", "s599", "s600",
    "s601", "s602", "s603", "s604", "s605", "s606", "s607", "s608", "s609", "s610", "s611", "s612",
    "s613", "s614", "s615", "s616", "s617", "s618", "s619", "s620", "s621", "s622", "s623", "s624",
    "s625", "s626", "s627", "s628", "s629", "s630", "s631", "s632", "s633", "s634", "s635", "s636",
    "s637", "s638", "s639", "s640", "s641", "s642", "s643", "s644", "s645", "s646", "s647", "s648",
    "s649", "s650", "s651", "s652", "s653", "s654", "s655", "s656", "s657", "s658", "s659", "s660",
    "s661", "s662", "s663", "s664", "s665", "s666", "s667", "s668", "s669", "s670", "s671", "s672",
    "s673", "s674", "s675", "s676", "s677", "s678", "s679", "s680", "s681", "s682", "s683", "s684",
    "s685", "s686", "s687", "s688", "s689", "s690", "s691", "s692", "s693", "s694", "s695", "s696",
    "s697", "s698", "s699", "s700", "s701", "s702", "s703", "s704", "s705", "s706", "s707", "s708",
    "s709", "s710", "s711", "s712", "s713", "s714", "s715", "s716", "s717", "s718", "s719", "s720",
    "s721", "s722", "s723", "s724", "s725", "s726", "s727", "s728", "s729", "s730", "s731", "s732",
    "s733", "s734", "s735", "s736", "s737", "s738", "s739", "s740", "s741", "s742", "s743", "s744",
    "s745", "s746", "s747", "s748", "s749", "s750", "s751", "s752", "s753", "s754", "s755", "s756",
    "s757", "s758", "s759", "s760", "s761", "s762", "s763", "s764", "s765", "s766", "s767", "s768",
    "s769", "s770", "s771", "s772", "s773", "s774", "s775", "s776", "s777", "s778", "s779", "s780",
    "s781", "s782", "s783", "s784", "s785", "s786", "s787", "s788", "s789", "s790", "s791", "s792",
    "s793", "s794", "s795", "s796", "s797", "s798", "s799", "s800", "s801", "s802", "s803", "s804",
    "s805", "s806", "s807", "s808", "s809", "s810", "s811", "s812", "s813", "s814", "s815", "s816",
    "s817", "s818", "s819", "s820", "s821", "s822", "s823", "s824", "s825", "s826", "s827", "s828",
    "s829", "s830", "s831", "s832", "s833", "s834", "s835", "s836", "s837", "s838", "s839", "s840",
    "s841", "s842", "s843", "s844", "s845", "s846", "s847", "s848", "s849", "s850", "s851", "s852",
    "s853", "s854", "s855", "s856", "s857", "s858", "s859", "s860", "s861", "s862", "s863", "s864",
    "s865", "s866", "s867", "s868", "s869", "s870", "s871", "s872", "s873", "s874", "s875", "s876",
    "s877", "s878", "s879", "s880", "s881", "s882", "s883", "s884", "s885", "s886", "s887", "s888",
    "s889", "s890", "s891", "s892", "s893", "s894", "s895", "s896", "s897", "s898", "s899", "s900"
  };

  Race r{ 10, 20, 6 };
  std::cout << "--------------------------- INIT RACE ---------------------------\n";
  r.InitRace(sporter_name, 1.0, 5.0);
  std::cout << "--------------------------- START RACE --------------------------\n";
  r.RaceStart();
  return 0;
}
