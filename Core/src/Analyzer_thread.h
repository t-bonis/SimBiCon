#pragma once
#include <QtCore/QThreadPool>

class Learning_framework;
class SimBiCon_framework;

class Analyzer_thread : public QThread
{
	Q_OBJECT
public:
	Analyzer_thread() = delete;
	Analyzer_thread(SimBiCon_framework& conF, bool print_result = false);
	~Analyzer_thread() = default;

public slots:
	
signals:
	void analyze_done(SimBiCon_framework* simbicon_framework);

private:
	void eval_simulation() const;
	void run() override;

	SimBiCon_framework* m_simbicon_framework;
	bool m_print_result;
};

