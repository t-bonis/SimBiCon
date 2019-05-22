#pragma once
#include <QtCore/QThreadPool>

class Learning_framework;
class SimBiCon_framework;

class Analyzer_thread : public QThread
{
	Q_OBJECT
public:
	Analyzer_thread() = delete;
	explicit Analyzer_thread(SimBiCon_framework& conF);
	Analyzer_thread(SimBiCon_framework& conF, Learning_framework& learnF);
	~Analyzer_thread() = default;

public slots:
	
signals:
	void analyze_done(SimBiCon_framework* simbicon_framework);

private:
	void eval_simulation() const;
	void eval_intervals() const;
	void run() override;

	SimBiCon_framework* m_simbicon_framework;
	Learning_framework* m_learning_framework = nullptr;
};

