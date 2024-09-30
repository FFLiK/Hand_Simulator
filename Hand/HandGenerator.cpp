#include "HandGenerator.h"
#include <thread>

Hand* HandGenerator::Generate() {
	return Generate_Hand();
}

Hand* HandGenerator::Generate_Hand() {
	double HL, HB;
	HL = 500;
	HB = 375;
	
	Hand* hand = new Hand();

	// ================================================
	
	//! Joint Assignment

	//  Base
	auto* base = (new Joint<JointType::TIRTARY>(0))->InitAngle(0, 0, 0);

	//  Carpals
	auto thumb_carpal = (new Joint<JointType::NONE>(0.25 * HL, base))->InitAngle(0, 0, 40);
	auto index_carpal = (new Joint<JointType::NONE>(0.25 * HL, base))->InitAngle(0, 0, 60);
	auto middle_carpal = (new Joint<JointType::NONE>(0.25 * HL, base))->InitAngle(0, 0, 80);
	auto ring_carpal = (new Joint<JointType::NONE>(0.25 * HL, base))->InitAngle(0, 0, 100);
	auto pinky_carpal = (new Joint<JointType::NONE>(0.25 * HL, base))->InitAngle(0, 0, 120);

	//  Metacarpas : Carpometacarpal (CMC)
	auto thumb_metacarpa_base = (new Joint<JointType::NONE>(0, thumb_carpal))->InitAngle(0, 0, 50);
	auto thumb_metacarpa = (Joint<JointType::TIRTARY>*)(new Joint<JointType::TIRTARY>(0.251 * HL, thumb_metacarpa_base))->InitAngle(0, 40, -40)->SetRangeY(0, 100)->SetRangeZ(-80, -10);
	auto index_metacarpa = (new Joint<>(sqrt(pow(0.374 * HL, 2) + pow(0.126 * HB, 2)), index_carpal))->InitAngle(0, 0, 15);
	auto middle_metacarpa = (new Joint<>(0.373 * HL, middle_carpal))->InitAngle(0, 0, 5);
	auto ring_metacarpa = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(sqrt(pow(0.336 * HL, 2) + pow(0.077 * HB, 2)), ring_carpal))->InitAngle(7, 0, -4)->SetRangeX(0, 10)->SetRangeZ(-7, -2);
	auto pinky_metacarpa = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(sqrt(pow(0.295 * HL, 2) + pow(0.179 * HB, 2)), pinky_carpal))->InitAngle(13, 0, -10)->SetRangeX(0, 20)->SetRangeZ(-18, -5);

	//  Phalanges : Metacarpophalangeal (MCP)
	auto thumb_phalange_first = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(0.196 * HL, thumb_metacarpa))->InitAngle(15, 30, 0)->SetRangeX(0, 90)->SetRangeZ(-15, 0);
	auto index_phalange_first = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(0.265 * HL, index_metacarpa))->InitAngle(50, 0, 0)->SetRangeX(-25, 90)->SetRangeZ(-20, 20);
	auto middle_phalange_first = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(0.277 * HL, middle_metacarpa))->InitAngle(45, 0, 0)->SetRangeX(-15, 90)->SetRangeZ(-20, 20);
	auto ring_phalange_first = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(0.259 * HL, ring_metacarpa))->InitAngle(30, 0, 0)->SetRangeX(-15, 90)->SetRangeZ(-20, 20);
	auto pinky_phalange_first = (Joint<JointType::SECONDARY>*)(new Joint<JointType::SECONDARY>(0.206 * HL, pinky_metacarpa))->InitAngle(15, 0, 0)->SetRangeX(-25, 90)->SetRangeZ(-20, 20);

	//  Phalanges : Proximal Interphalangeal (PIP)
	auto index_phalange_second = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.143 * HL, index_phalange_first))->InitAngle(30, 0, 0)->SetRangeX(-2, 110);
	auto middle_phalange_second = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.170 * HL, middle_phalange_first))->InitAngle(45, 0, 0)->SetRangeX(-2, 110);
	auto ring_phalange_second = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.165 * HL, ring_phalange_first))->InitAngle(55, 0, 0)->SetRangeX(-2, 110);
	auto pinky_phalange_second = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.117 * HL, pinky_phalange_first))->InitAngle(60, 0, 0)->SetRangeX(-2, 110);

	//  Phalanges : Distal Interphalangeal (DIP)
	auto thumb_phalange_third = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.158 * HL, thumb_phalange_first))->InitAngle(40, 0, 0)->SetFlag(JointFlag::END_EFFECTOR)->SetRangeX(-10, 90);
	auto index_phalange_third = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.097 * HL, index_phalange_second))->InitAngle(10, 0, 0)->SetFlag(JointFlag::END_EFFECTOR)->SetRangeX(-5, 80);
	auto middle_phalange_third = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.108 * HL, middle_phalange_second))->InitAngle(10, 0, 0)->SetFlag(JointFlag::END_EFFECTOR)->SetRangeX(-5, 80);
	auto ring_phalange_third = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.107 * HL, ring_phalange_second))->InitAngle(10, 0, 0)->SetFlag(JointFlag::END_EFFECTOR)->SetRangeX(-5, 80);
	auto pinky_phalange_third = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(0.093 * HL, pinky_phalange_second))->InitAngle(10, 0, 0)->SetFlag(JointFlag::END_EFFECTOR)->SetRangeX(-5, 80);

	//  Wrist Flesh
	auto wrist_flesh_1 = (new Joint<JointType::NONE>(0.125 * HL, base))->InitAngle(0, 0, 0)->SetFlag(JointFlag::VIRTUAL_JOINT);
	auto wrist_flesh_2 = (new Joint<JointType::NONE>(0.125 * HL, base))->InitAngle(0, 0, 180)->SetFlag(JointFlag::VIRTUAL_JOINT);

	// ================================================

	//! Muscle Assignment
	
	//! From the Arm

	#define XZ MuscleMotionDirection::COS, MuscleMotionDirection::ZERO, MuscleMotionDirection::SIN
	#define YZ MuscleMotionDirection::ZERO, MuscleMotionDirection::COS, MuscleMotionDirection::SIN

	//  Flexor Digitorum Superficialis : ¾èÀº ¼Õ°¡¶ô ±ÁÈù±Ù
	auto flexor_digitorum_superficialis = (new Muscle())->SetMuscleUnitSize(4)->SetMuscleStrength(1.0);
	flexor_digitorum_superficialis->GetMuscleUnit(0)->AddJoint(index_carpal, 0, 0, XZ)->AddJoint(index_metacarpa, 0, 10, XZ)->AddJoint(index_phalange_first, 0, 10, XZ)->AddJoint(index_phalange_second, 0, 0, XZ);
	flexor_digitorum_superficialis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);
	flexor_digitorum_superficialis->GetMuscleUnit(1)->AddJoint(middle_carpal, 0, 0, XZ)->AddJoint(middle_metacarpa, 0, 10, XZ)->AddJoint(middle_phalange_first, 0, 10, XZ)->AddJoint(middle_phalange_second, 0, 0, XZ);
	flexor_digitorum_superficialis->GetMuscleUnit(1)->SetContractingAngleSummation(-1);
	flexor_digitorum_superficialis->GetMuscleUnit(2)->AddJoint(ring_carpal, 0, 0, XZ)->AddJoint(ring_metacarpa, 0, 10, XZ)->AddJoint(ring_phalange_first, 0, 10, XZ)->AddJoint(ring_phalange_second, 0, 0, XZ);
	flexor_digitorum_superficialis->GetMuscleUnit(2)->SetContractingAngleSummation(-1);
	flexor_digitorum_superficialis->GetMuscleUnit(3)->AddJoint(pinky_carpal, 0, 0, XZ)->AddJoint(pinky_metacarpa, 0, 10, XZ)->AddJoint(pinky_phalange_first, 0, 10, XZ)->AddJoint(pinky_phalange_second, 0, 0, XZ);
	flexor_digitorum_superficialis->GetMuscleUnit(3)->SetContractingAngleSummation(-1);

	//  Flexor Digitorum Profundus : ±íÀº ¼Õ°¡¶ô ±ÁÈù±Ù
	auto flexor_digitorum_profundus = (new Muscle())->SetMuscleUnitSize(4)->SetMuscleStrength(1.0);
	flexor_digitorum_profundus->GetMuscleUnit(0)->AddJoint(index_carpal, 0, 0, XZ)->AddJoint(index_metacarpa, 0, 20, XZ)->AddJoint(index_phalange_first, 0, 20, XZ)->AddJoint(index_phalange_second, 0, 20, XZ)->AddJoint(index_phalange_third, 0, 0, XZ);
	flexor_digitorum_profundus->GetMuscleUnit(0)->SetContractingAngleSummation(-1);
	flexor_digitorum_profundus->GetMuscleUnit(1)->AddJoint(middle_carpal, 0, 0, XZ)->AddJoint(middle_metacarpa, 0, 20, XZ)->AddJoint(middle_phalange_first, 0, 20, XZ)->AddJoint(middle_phalange_second, 0, 20, XZ)->AddJoint(middle_phalange_third, 0, 0, XZ);
	flexor_digitorum_profundus->GetMuscleUnit(1)->SetContractingAngleSummation(-1);
	flexor_digitorum_profundus->GetMuscleUnit(2)->AddJoint(ring_carpal, 0, 0, XZ)->AddJoint(ring_metacarpa, 0, 20, XZ)->AddJoint(ring_phalange_first, 0, 20, XZ)->AddJoint(ring_phalange_second, 0, 20, XZ)->AddJoint(ring_phalange_third, 0, 0, XZ);
	flexor_digitorum_profundus->GetMuscleUnit(2)->SetContractingAngleSummation(-1);
	flexor_digitorum_profundus->GetMuscleUnit(3)->AddJoint(pinky_carpal, 0, 0, XZ)->AddJoint(pinky_metacarpa, 0, 20, XZ)->AddJoint(pinky_phalange_first, 0, 20, XZ)->AddJoint(pinky_phalange_second, 0, 20, XZ)->AddJoint(pinky_phalange_third, 0, 0, XZ);
	flexor_digitorum_profundus->GetMuscleUnit(3)->SetContractingAngleSummation(-1);

	//  Extensor Digitorum : ¼Õ°¡¶ô Æï±Ù
	auto extensor_digitorum = (new Muscle())->SetMuscleUnitSize(4)->SetMuscleStrength(1.0);
	extensor_digitorum->GetMuscleUnit(0)->AddJoint(index_carpal, 180, 0, XZ)->AddJoint(index_metacarpa, 180, 20, XZ)->AddJoint(index_phalange_first, 180, 20, XZ)->AddJoint(index_phalange_second, 180, 20, XZ)->AddJoint(index_phalange_third, 180, 0, XZ);
	extensor_digitorum->GetMuscleUnit(0)->SetContractingAngleSummation(-1);
	extensor_digitorum->GetMuscleUnit(1)->AddJoint(middle_carpal, 180, 0, XZ)->AddJoint(middle_metacarpa, 180, 20, XZ)->AddJoint(middle_phalange_first, 180, 20, XZ)->AddJoint(middle_phalange_second, 180, 20, XZ)->AddJoint(middle_phalange_third, 180, 0, XZ);
	extensor_digitorum->GetMuscleUnit(1)->SetContractingAngleSummation(-1);
	extensor_digitorum->GetMuscleUnit(2)->AddJoint(ring_carpal, 180, 0, XZ)->AddJoint(ring_metacarpa, 180, 20, XZ)->AddJoint(ring_phalange_first, 180, 20, XZ)->AddJoint(ring_phalange_second, 180, 20, XZ)->AddJoint(ring_phalange_third, 180, 0, XZ);
	extensor_digitorum->GetMuscleUnit(2)->SetContractingAngleSummation(-1);
	extensor_digitorum->GetMuscleUnit(3)->AddJoint(pinky_carpal, 180, 0, XZ)->AddJoint(pinky_metacarpa, 180, 20, XZ)->AddJoint(pinky_phalange_first, 180, 20, XZ)->AddJoint(pinky_phalange_second, 180, 20, XZ)->AddJoint(pinky_phalange_third, 180, 0, XZ);
	extensor_digitorum->GetMuscleUnit(3)->SetContractingAngleSummation(-1);

	//  Flexor Pollicis Longus : ±ä¾öÁö±ÁÈû±Ù
	auto flexor_pollicis_longus = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	flexor_pollicis_longus->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(thumb_carpal, 0, 20, XZ)->AddJoint(thumb_metacarpa, 0, 20, YZ)->AddJoint(thumb_phalange_first, 0, 20, XZ)->AddJoint(thumb_phalange_third, 0, 0, XZ);
	flexor_pollicis_longus->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	//  Aductor Pollicis Longus : ±ä¾öÁö¹ú¸²±Ù
	auto aductor_pollicis_longus = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	aductor_pollicis_longus->GetMuscleUnit(0)->AddJoint(base, 240, 0, XZ)->AddJoint(thumb_carpal, 240, 20, XZ)->AddJoint(thumb_metacarpa, 240, 0, YZ);
	aductor_pollicis_longus->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	//  Extensor Pollicis Longus : ±ä¾öÁöÆï±Ù
	auto extensor_pollicis_longus = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	extensor_pollicis_longus->GetMuscleUnit(0)->AddJoint(base, 180, 0, XZ)->AddJoint(thumb_carpal, 180, 20, XZ)->AddJoint(thumb_metacarpa, 180, 20, YZ)->AddJoint(thumb_phalange_first, 180, 20, XZ)->AddJoint(thumb_phalange_third, 180, 0, XZ);
	extensor_pollicis_longus->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	//  Extensor Pollicis Brevis : ÂªÀº¾öÁöÆï±Ù
	auto extensor_pollicis_brevis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	extensor_pollicis_brevis->GetMuscleUnit(0)->AddJoint(base, 180, 0, XZ)->AddJoint(thumb_carpal, 180, 10, XZ)->AddJoint(thumb_metacarpa, 180, 10, YZ)->AddJoint(thumb_phalange_first, 180, 0, XZ);
	extensor_pollicis_brevis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	//  Extensor Indicis : °ËÁöÆï±Ù
	auto extensor_indicis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	extensor_indicis->GetMuscleUnit(0)->AddJoint(index_carpal, 180, 0, XZ)->AddJoint(index_metacarpa, 180, 10, XZ)->AddJoint(index_phalange_first, 180, 10, XZ)->AddJoint(index_phalange_second, 180, 10, XZ)->AddJoint(index_phalange_third, 180, 0, XZ);
	extensor_indicis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);
	
	//  Extensor Digiti Minimi : »õ³¢Æï±Ù
	auto extensor_digiti_minimi = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	extensor_digiti_minimi->GetMuscleUnit(0)->AddJoint(pinky_carpal, 180, 0, XZ)->AddJoint(pinky_metacarpa, 180, 10, XZ)->AddJoint(pinky_phalange_first, 180, 10, XZ)->AddJoint(pinky_phalange_second, 180, 10, XZ)->AddJoint(pinky_phalange_third, 180, 0, XZ);
	extensor_digiti_minimi->GetMuscleUnit(0)->SetContractingAngleSummation(-1);


	//! From the Hand
	
	// Aductor Pollicis Brevis : ÂªÀº¾öÁö¹ú¸²±Ù
	auto aductor_pollicis_brevis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	aductor_pollicis_brevis->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(thumb_carpal, -20, 30, XZ)->AddJoint(thumb_metacarpa, -70, 20, YZ)->AddJoint(thumb_phalange_first, -90, 0, XZ);
	aductor_pollicis_brevis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Flexor Pollicis Brevis : ÂªÀº¾öÁö±ÁÈû±Ù
	auto flexor_pollicis_brevis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	flexor_pollicis_brevis->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(thumb_carpal, 0, 20, XZ)->AddJoint(thumb_metacarpa, 0, 20, YZ)->AddJoint(thumb_phalange_first, 0, 0, XZ);
	flexor_pollicis_brevis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Opponens Pollicis : ¾öÁö¸Â¼¶±Ù
	auto opponens_pollicis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	opponens_pollicis->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(thumb_carpal, 20, 20, XZ)->AddJoint(thumb_metacarpa, 20, 0, YZ);
	opponens_pollicis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Adductor Pollicis : ¾öÁö¸ðÀ½±Ù
	auto adductor_pollicis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	adductor_pollicis->GetMuscleUnit(0)->AddJoint(index_carpal, 0, 0, XZ)->AddJoint(thumb_metacarpa, 20, 20, YZ)->AddJoint(middle_carpal, 0, 0, XZ);
	adductor_pollicis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Adductor Digiti Minimi : »õ³¢¹ú¸²±Ù
	auto adductor_digiti_minimi = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	adductor_digiti_minimi->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(pinky_carpal, 80, 20, XZ)->AddJoint(pinky_metacarpa, 80, 0, XZ);
	adductor_digiti_minimi->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Flexor Digiti Minimi Brevis : ÂªÀº»õ³¢±ÁÈû±Ù
	auto flexor_digiti_minimi_brevis = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	flexor_digiti_minimi_brevis->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(pinky_carpal, 0, 20, XZ)->AddJoint(pinky_metacarpa, 0, 20, XZ)->AddJoint(pinky_phalange_first, 0, 0, XZ);
	flexor_digiti_minimi_brevis->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// Opponens Digiti Minimi : »õ³¢¸Â¼¶±Ù
	auto opponens_digiti_minimi = (new Muscle())->SetMuscleUnitSize(1)->SetMuscleStrength(1.0);
	opponens_digiti_minimi->GetMuscleUnit(0)->AddJoint(base, 0, 0, XZ)->AddJoint(pinky_carpal, -60, 20, XZ)->AddJoint(pinky_metacarpa, -60, 20, XZ)->AddJoint(pinky_phalange_first, -60, 0, XZ);
	opponens_digiti_minimi->GetMuscleUnit(0)->SetContractingAngleSummation(-1);

	// ================================================

	// Assign Joints to Hand
	hand->AddJoint(base);
	hand->AddJoint(wrist_flesh_1);
	hand->AddJoint(thumb_carpal)->AddJoint(thumb_metacarpa_base)->AddJoint(thumb_metacarpa)->AddJoint(thumb_phalange_first)->AddJoint(thumb_phalange_third);
	hand->AddJoint(index_carpal)->AddJoint(index_metacarpa)->AddJoint(index_phalange_first)->AddJoint(index_phalange_second)->AddJoint(index_phalange_third);
	hand->AddJoint(middle_carpal)->AddJoint(middle_metacarpa)->AddJoint(middle_phalange_first)->AddJoint(middle_phalange_second)->AddJoint(middle_phalange_third);
	hand->AddJoint(ring_carpal)->AddJoint(ring_metacarpa)->AddJoint(ring_phalange_first)->AddJoint(ring_phalange_second)->AddJoint(ring_phalange_third);
	hand->AddJoint(pinky_carpal)->AddJoint(pinky_metacarpa)->AddJoint(pinky_phalange_first)->AddJoint(pinky_phalange_second)->AddJoint(pinky_phalange_third);
	hand->AddJoint(wrist_flesh_2);

	// Assign Muscles to Hand
	hand->AddMuscle(flexor_digitorum_superficialis);
	hand->AddMuscle(flexor_digitorum_profundus);
	hand->AddMuscle(extensor_digitorum);
	hand->AddMuscle(flexor_pollicis_longus);
	hand->AddMuscle(aductor_pollicis_longus);
	hand->AddMuscle(extensor_pollicis_longus);
	hand->AddMuscle(extensor_pollicis_brevis);
	hand->AddMuscle(extensor_indicis);
	hand->AddMuscle(extensor_digiti_minimi);

	hand->AddMuscle(aductor_pollicis_brevis);
	hand->AddMuscle(flexor_pollicis_brevis);
	hand->AddMuscle(opponens_pollicis);
	hand->AddMuscle(adductor_pollicis);
	hand->AddMuscle(adductor_digiti_minimi);
	hand->AddMuscle(flexor_digiti_minimi_brevis);
	hand->AddMuscle(opponens_digiti_minimi);

	// Assign False Lines to Hand
	hand->AddFalseLine(wrist_flesh_1, thumb_carpal);
	hand->AddFalseLine(wrist_flesh_1, thumb_metacarpa);
	hand->AddFalseLine(wrist_flesh_2, pinky_carpal);
	hand->AddFalseLine(wrist_flesh_2, pinky_metacarpa);
	hand->AddFalseLine(thumb_carpal, index_carpal);
	hand->AddFalseLine(index_carpal, middle_carpal);
	hand->AddFalseLine(middle_carpal, ring_carpal);
	hand->AddFalseLine(ring_carpal, pinky_carpal);
	hand->AddFalseLine(thumb_metacarpa, index_metacarpa);
	hand->AddFalseLine(index_metacarpa, middle_metacarpa);
	hand->AddFalseLine(middle_metacarpa, ring_metacarpa);
	hand->AddFalseLine(ring_metacarpa, pinky_metacarpa);

	auto POWER_UNIT = 1.0;

	auto press_function_generator([=](Muscle* muscle) {
		return new function<void()>([=]() {
			double power = muscle->GetPower();
			power += POWER_UNIT;
			if (power > 1.0) power = 1.0;
			muscle->SetPower(power);
			});
		}
	);

	auto release_function_generator([=](Muscle* muscle) {
		return new function<void()>([=]() {
			double power = muscle->GetPower();
			power -= POWER_UNIT;
			if (power < 0.0) power = 0.0;
			muscle->SetPower(power);
			});
		}
	);

	hand->press_motion_function_set.push_back(press_function_generator(adductor_digiti_minimi));
	hand->release_motion_function_set.push_back(release_function_generator(adductor_digiti_minimi));

	hand->press_motion_function_set.push_back(press_function_generator(flexor_digiti_minimi_brevis));
	hand->release_motion_function_set.push_back(release_function_generator(flexor_digiti_minimi_brevis));

	hand->press_motion_function_set.push_back(press_function_generator(opponens_digiti_minimi));
	hand->release_motion_function_set.push_back(release_function_generator(opponens_digiti_minimi));

	return hand;
}


Hand* HandGenerator::Generate_Test() {
	Hand* hand = new Hand();

	hand->SetOrientation(0, 0, 0);

	auto* base = (new Joint<JointType::TIRTARY>(0))->InitAngle(0, 0, 90);

	auto* left = (new Joint<JointType::NONE>(100, base))->InitAngle(0, 0, 90);
	auto* right = (new Joint<JointType::NONE>(100, base))->InitAngle(0, 0, -90);

	auto left_base = (new Joint<JointType::NONE>(0, left))->InitAngle(0, 0, -90);
	auto right_base = (new Joint<JointType::NONE>(0, right))->InitAngle(0, 0, 90);

	auto first_finger_first_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(200, left_base))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto first_finger_second_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(180, first_finger_first_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto first_finger_third_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(150, first_finger_second_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::END_EFFECTOR);

	auto second_finger_first_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(200, base))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto second_finger_second_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(180, second_finger_first_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto second_finger_third_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(150, second_finger_second_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::END_EFFECTOR);

	auto third_finger_first_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(200, right_base))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto third_finger_second_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(180, third_finger_first_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::REAL_JOINT);
	auto third_finger_third_joint = (Joint<JointType::PRIMARY>*)(new Joint<JointType::PRIMARY>(150, third_finger_second_joint))->InitAngle(30, 0, 0)->SetRangeX(-5, 110)->SetFlag(JointFlag::END_EFFECTOR);


	hand->AddJoint(base);
	hand->AddJoint(left);
	hand->AddJoint(right);
	hand->AddJoint(left_base);
	hand->AddJoint(right_base);
	hand->AddJoint(first_finger_first_joint);
	hand->AddJoint(first_finger_second_joint);
	hand->AddJoint(first_finger_third_joint);
	hand->AddJoint(second_finger_first_joint);
	hand->AddJoint(second_finger_second_joint);
	hand->AddJoint(second_finger_third_joint);
	hand->AddJoint(third_finger_first_joint);
	hand->AddJoint(third_finger_second_joint);
	hand->AddJoint(third_finger_third_joint);

	auto inner_muscle = (new Muscle())->SetMuscleUnitSize(3)->SetMuscleStrength(1.0);
	inner_muscle->GetMuscleUnit(0)->AddJoint(left_base, 0, 0, XZ)->AddJoint(first_finger_first_joint, 0, 30, XZ)->AddJoint(first_finger_second_joint, 0, 30, XZ)->AddJoint(first_finger_third_joint, 0, 0, XZ);
	inner_muscle->GetMuscleUnit(0)->SetContractingAngleSummation(350);
	inner_muscle->GetMuscleUnit(1)->AddJoint(base, 0, 0, XZ)->AddJoint(second_finger_first_joint, 0, 30, XZ)->AddJoint(second_finger_second_joint, 0, 30, XZ)->AddJoint(second_finger_third_joint, 0, 0, XZ);
	inner_muscle->GetMuscleUnit(1)->SetContractingAngleSummation(350);
	inner_muscle->GetMuscleUnit(2)->AddJoint(right_base, 0, 0, XZ)->AddJoint(third_finger_first_joint, 0, 30, XZ)->AddJoint(third_finger_second_joint, 0, 30, XZ)->AddJoint(third_finger_third_joint, 0, 0, XZ);
	inner_muscle->GetMuscleUnit(2)->SetContractingAngleSummation(350);
	
	auto outer_muscle = (new Muscle())->SetMuscleUnitSize(3)->SetMuscleStrength(1.0);
	outer_muscle->GetMuscleUnit(0)->AddJoint(left_base, 0, 0, XZ)->AddJoint(first_finger_first_joint, 180, 30, XZ)->AddJoint(first_finger_second_joint, 180, 30, XZ)->AddJoint(first_finger_third_joint, 180, 0, XZ);
	outer_muscle->GetMuscleUnit(0)->SetContractingAngleSummation(30);
	outer_muscle->GetMuscleUnit(1)->AddJoint(base, 0, 0, XZ)->AddJoint(second_finger_first_joint, 180, 30, XZ)->AddJoint(second_finger_second_joint, 180, 30, XZ)->AddJoint(second_finger_third_joint, 180, 0, XZ);
	outer_muscle->GetMuscleUnit(1)->SetContractingAngleSummation(30);
	outer_muscle->GetMuscleUnit(2)->AddJoint(right_base, 0, 0, XZ)->AddJoint(third_finger_first_joint, 180, 30, XZ)->AddJoint(third_finger_second_joint, 180, 30, XZ)->AddJoint(third_finger_third_joint, 180, 0, XZ);
	outer_muscle->GetMuscleUnit(2)->SetContractingAngleSummation(30);

	auto inner_sub_muscle = (new Muscle())->SetMuscleUnitSize(3)->SetMuscleStrength(1.0);
	inner_sub_muscle->GetMuscleUnit(0)->AddJoint(left_base, 0, 0, XZ)->AddJoint(first_finger_first_joint, 0, 15, XZ)->AddJoint(first_finger_second_joint, 0, 0, XZ);
	inner_sub_muscle->GetMuscleUnit(0)->SetContractingAngleSummation(250);
	inner_sub_muscle->GetMuscleUnit(1)->AddJoint(base, 0, 0, XZ)->AddJoint(second_finger_first_joint, 0, 15, XZ)->AddJoint(second_finger_second_joint, 0, 0, XZ);
	inner_sub_muscle->GetMuscleUnit(1)->SetContractingAngleSummation(250);
	inner_sub_muscle->GetMuscleUnit(2)->AddJoint(right_base, 0, 0, XZ)->AddJoint(third_finger_first_joint, 0, 15, XZ)->AddJoint(third_finger_second_joint, 0, 0, XZ);
	inner_sub_muscle->GetMuscleUnit(2)->SetContractingAngleSummation(250);

	hand->AddMuscle(inner_muscle);
	hand->AddMuscle(outer_muscle);
	hand->AddMuscle(inner_sub_muscle);

	double POWER_UNIT = 1.0;

	hand->press_motion_function_set.push_back(new function<void()>([=]() {
			double power = inner_muscle->GetPower();
			power += POWER_UNIT;
			if (power > 1.0) power = 1.0;
			inner_muscle->SetPower(power, 0);
		})
	);

	hand->release_motion_function_set.push_back(new function<void()>([=]() {
			double power = inner_muscle->GetPower();
			power -= POWER_UNIT;
			if (power < 0.0) power = 0.0;
			inner_muscle->SetPower(power, 0);
		})
	);

	hand->press_motion_function_set.push_back(new function<void()>([=]() {
			double power = outer_muscle->GetPower();
			power += POWER_UNIT;
			if (power > 1.0) power = 1.0;
			outer_muscle->SetPower(power, 0);
		})
	);

	hand->release_motion_function_set.push_back(new function<void()>([=]() {
			double power = outer_muscle->GetPower();
			power -= POWER_UNIT;
			if (power < 0.0) power = 0.0;
			outer_muscle->SetPower(power, 0);
		})
	);

	hand->press_motion_function_set.push_back(new function<void()>([=]() {
			double power = inner_sub_muscle->GetPower();
			power += POWER_UNIT;
			if (power > 1.0) power = 1.0;
			inner_sub_muscle->SetPower(power, 0);
		})
	);

	hand->release_motion_function_set.push_back(new function<void()>([=]() {
			double power = inner_sub_muscle->GetPower();
			power -= POWER_UNIT;
			if (power < 0.0) power = 0.0;
			inner_sub_muscle->SetPower(power, 0);
		})
	);

	return hand;
}
