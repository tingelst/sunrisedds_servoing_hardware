#include <dds/dds.h>
#include "JointPosition.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#define MAX_SAMPLES 1

bool g_stop = false;

void signal_handler(int signum)
{
  g_stop = true;
  printf("Stopping!\n");
}

int main(int argc, char const* argv[])
{
  dds_entity_t participant;
  dds_entity_t command_topic;
  dds_entity_t state_topic;
  dds_entity_t reader;
  dds_entity_t writer;
  sunrisedds_interfaces_msg_JointPosition* command_msg;
  sunrisedds_interfaces_msg_JointPosition state_msg;
  void* samples[MAX_SAMPLES];
  dds_sample_info_t infos[MAX_SAMPLES];
  dds_return_t rc;
  dds_qos_t* qos;
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  signal(SIGINT, signal_handler);

  /* Create a Participant. */
  participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  command_topic =
      dds_create_topic(participant, &sunrisedds_interfaces_msg_JointPosition_desc, "rt/command", NULL, NULL);
  if (command_topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-command_topic));

  /* Create a Topic. */
  state_topic = dds_create_topic(participant, &sunrisedds_interfaces_msg_JointPosition_desc, "rt/state", NULL, NULL);
  if (command_topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-command_topic));

  /* Create a reliable Reader. */
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
  reader = dds_create_reader(participant, command_topic, qos, NULL);
  if (reader < 0)
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  /* Create a Writer. */
  writer = dds_create_writer(participant, state_topic, NULL, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

  samples[0] = sunrisedds_interfaces_msg_JointPosition__alloc();

  state_msg.position.a1 = 0.5;
  state_msg.position.a2 = 0.5;
  state_msg.position.a3 = 0.5;
  state_msg.position.a4 = 0.5;
  state_msg.position.a5 = 0.5;
  state_msg.position.a6 = 0.5;
  state_msg.position.a7 = 0.5;

  /* Poll until data has been read. */
  while (!g_stop)
  {
    rc = dds_write(writer, &state_msg);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

    /* Do the actual read.
     * The return value contains the number of read samples. */
    rc = dds_read(reader, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
    if (rc < 0)
      DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));

    /* Check if we read some data and it is valid. */
    if ((rc > 0) && (infos[0].valid_data))
    {
      /* Print Message. */
      command_msg = (sunrisedds_interfaces_msg_JointPosition*)samples[0];

      state_msg.position.a1 = command_msg->position.a1;
      state_msg.position.a2 = command_msg->position.a2;
      state_msg.position.a3 = command_msg->position.a3;
      state_msg.position.a4 = command_msg->position.a4;
      state_msg.position.a5 = command_msg->position.a5;
      state_msg.position.a6 = command_msg->position.a6;
      state_msg.position.a7 = command_msg->position.a7;
    }

    dds_sleepfor(DDS_MSECS(10));
  }

  /* Free the data location. */
  sunrisedds_interfaces_msg_JointPosition_free(samples[0], DDS_FREE_ALL);

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  printf("Stopped\n");

  return EXIT_SUCCESS;
}
