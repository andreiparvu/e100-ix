// Andrei Parvu
// 341C3

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/slab.h>

#include <linux/pci.h>
#include <linux/etherdevice.h>

#include <linux/interrupt.h>

#include <asm/uaccess.h>

MODULE_DESCRIPTION("E100 driver");
MODULE_AUTHOR("Andrei Parvu");
MODULE_LICENSE("GPL");

#define LOG_LEVEL KERN_ALERT

#define MAX_CBS 10

#define CMD_BASE 0x02
#define SCB_POINTER_BASE 0x04

#define CUC_START (0b0001 << 4) | 0xbc00 // cmd start + interuperi doar pt
					 // receive
#define CUC_RESUME (0b0010 << 4) | 0xbc00 // cmd resume + intreruperi doar pt
					  // receive
#define CUC_LOAD_BASE (0b0110 << 4)

#define RUC_START 0b0001
#define RUC_RESUME 0b0010
#define RUC_LOAD_BASE 0b0110

#define CB_SEND 0x4004 // send + suspend bit
#define S_BIT (1 << 14)

#define FR_BIT (1 << 14)

#define TRUE 1
#define FALSE 0

#define MAX_ETHERNET_LENGTH 1518

// CB structure
struct e100_cb {
	__le16 status;
	__le16 cmd;
	__le32 link;

	u32 tbd_array;
	u16 tcb_byte_count;
	u8 threshold;
	u8 tbd_count;

	uint8_t data[MAX_ETHERNET_LENGTH];

	dma_addr_t dma_addr;
	struct e100_cb *next, *prev;
};

// Data for the driver
struct e100_data {
	struct e100_cb *tx_cbl[MAX_CBS];
	struct e100_cb *rx_cbl[MAX_CBS];

	struct pci_dev *pdev;
	struct net_device *net_dev;

	struct e100_cb *cur_tx_cb;
	struct e100_cb *cur_rx_cb;

	int has_started;
};

DEFINE_SPINLOCK(xmit_lock);

#define CMD_TIMEOUT 20000
#define SMALL_TIMEOUT 20

// Set a command to the command unit
void set_cmd(unsigned long base, unsigned short value, int no_wait) {
	int i;

	for (i = 0; i < CMD_TIMEOUT; i++) {
		unsigned char v = inw(base);

		// Wait for an empty register - it finished processing the last
		// command
		if (!v || no_wait) {
			outw(value, base);

			break;
		}

		if (i > SMALL_TIMEOUT) {
			udelay(5);
		}
	}
}

irqreturn_t e100_irq(int irq_no, void *dev_id)
{
	struct e100_data *data = (struct e100_data*)dev_id;
	struct sk_buff *skb;
	unsigned short status = inw(data->net_dev->base_addr);

	if ((status & FR_BIT) > 0) { // if receive
		// Alloc a new skb structure
		skb = dev_alloc_skb(data->cur_rx_cb->tcb_byte_count + 2);

		// Copy the data from the cb to the skb structure
		memcpy(skb_put(skb, data->cur_rx_cb->tcb_byte_count),
				data->cur_rx_cb->data,
				data->cur_rx_cb->tcb_byte_count);

		skb->dev = data->net_dev;
		skb->protocol = eth_type_trans(skb, data->net_dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		netif_rx(skb);

		// advance to next cb
		data->cur_rx_cb = data->cur_rx_cb->next;

		// signal that interrupt was handled
		set_cmd(data->net_dev->base_addr, 0xFF00, TRUE);
		// resume reception
		set_cmd(data->net_dev->base_addr + CMD_BASE, RUC_RESUME, FALSE);
	} else {
		// signal that interrupt was handled
		set_cmd(data->net_dev->base_addr, 0xFF00, TRUE);
	}

	return IRQ_HANDLED;
}

int e100_open(struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);
	int ret;

	// assign random mac
	net_dev->addr_assign_type = NET_ADDR_RANDOM;
	eth_random_addr(net_dev->dev_addr);

	// base address for control block
	net_dev->base_addr = data->pdev->resource[1].start;

	outw(0, net_dev->base_addr + CMD_BASE);

	// Load address of first CB
	data->cur_tx_cb = data->tx_cbl[0];
	data->cur_rx_cb = data->rx_cbl[0];

	outl(cpu_to_le32(data->cur_rx_cb->dma_addr), net_dev->base_addr +
			SCB_POINTER_BASE);

	set_cmd(net_dev->base_addr + CMD_BASE, RUC_START, FALSE);

	data->has_started = FALSE;
	ret = request_irq(net_dev->irq, e100_irq, IRQF_SHARED, "e100", data);

	return 0;
}

netdev_tx_t e100_start_xmit(struct sk_buff *skb, struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);
	struct e100_cb *cur_cb;
	int has_started;

	spin_lock(&xmit_lock);
	cur_cb = data->cur_tx_cb;

	// mark previous cb suspend bit as false
	cur_cb->prev->cmd &= ~S_BIT;
	// mark send for current cb
	cur_cb->cmd = cpu_to_le16(CB_SEND | S_BIT);
	has_started = data->has_started;
	// move current cb
	data->cur_tx_cb = data->cur_tx_cb->next;
	spin_unlock(&xmit_lock);

	memcpy(cur_cb->data, skb->data, skb->len);
	cur_cb->tcb_byte_count = skb->len;

	if (has_started == FALSE) {
		data->has_started = TRUE;
		outl(cpu_to_le32(cur_cb->dma_addr), net_dev->base_addr +
				SCB_POINTER_BASE);
		set_cmd(net_dev->base_addr + CMD_BASE, CUC_START, FALSE);
	} else {
		set_cmd(net_dev->base_addr + CMD_BASE, CUC_RESUME, FALSE);
	}

	return NETDEV_TX_OK;
}

int e100_stop(struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);

	netif_stop_queue(net_dev);
	free_irq(net_dev->irq, data);

	return 0;
}

static int e100_set_mac_address(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	return 0;
}

struct net_device_ops e100_ops = {
	.ndo_open = e100_open,
	.ndo_stop = e100_stop,
	.ndo_start_xmit = e100_start_xmit,
	.ndo_set_mac_address = e100_set_mac_address,
};

static void alloc_cbl(struct e100_cb *cbl[MAX_CBS], struct pci_dev *dev) {
	int i;

	for (i = 0; i < MAX_CBS; i++) {
		dma_addr_t dma_addr;

		// alloc structure for cb
		cbl[i] = pci_alloc_consistent(dev,
				sizeof(struct e100_cb), &dma_addr);
		cbl[i]->dma_addr = dma_addr;
		cbl[i]->tbd_array = 0xffffffff;
		cbl[i]->threshold = 0xe0;
		cbl[i]->tbd_count = 0;
		cbl[i]->cmd = 0;

		// make the links
		if (i > 0) {
			cbl[i - 1]->link = cpu_to_le32(dma_addr);
			cbl[i - 1]->next = cbl[i];
			cbl[i]->prev = cbl[i - 1];
		}
		if (i == MAX_CBS - 1) {
			cbl[i]->link =
				cpu_to_le32(cbl[0]->dma_addr);
			cbl[i]->next = cbl[0];
			cbl[0]->prev = cbl[i];
		}
	}
}

static void free_cbl(struct e100_cb *cbl[MAX_CBS], struct pci_dev *dev) {
	int i;

	for (i = 0; i < MAX_CBS; i++) {
		pci_free_consistent(dev, sizeof(struct e100_cb), cbl[i],
				cbl[i]->dma_addr);
	}
}

	
int e100_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;
	struct net_device *net_dev = alloc_etherdev(sizeof(struct e100_data));
	struct e100_data *data = netdev_priv(net_dev);

	ret = pci_enable_device(dev);

	if (ret < 0) {
		return ret;
	}

	pci_set_master(dev);

	data->pdev = dev;
	data->net_dev = net_dev;

	strcpy(net_dev->name, "ixeth%d");
	net_dev->netdev_ops = &e100_ops;

	ret = register_netdev(net_dev);

	if (ret < 0) {
		return ret;
	}

	net_dev->irq = dev->irq;

	alloc_cbl(data->tx_cbl, dev);
	alloc_cbl(data->rx_cbl, dev);

	return 0;
}

void e100_remove(struct pci_dev *dev)
{
	struct net_device *net_dev = pci_get_drvdata(dev);
	struct e100_data *data = netdev_priv(net_dev);

	if (net_dev) {
		free_cbl(data->tx_cbl, dev);
		free_cbl(data->rx_cbl, dev);

		unregister_netdev(net_dev);
		free_netdev(net_dev);
	}
}

struct pci_device_id e100_id_table[] = {
	{
		.vendor = 0x8086,
		.device = 0x1209,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID
	},
	{0,}
};

struct pci_driver e100_driver = {
	.name = "e100-ix",
	.id_table = e100_id_table,
	.probe = e100_probe,
	.remove = e100_remove
};

static int e100_init(void)
{
	int ret = pci_register_driver(&e100_driver);

	return ret;
}

static void e100_exit(void)
{
	pci_unregister_driver(&e100_driver);
}

module_init(e100_init);
module_exit(e100_exit);
